/*
 * Copyright 2010-2013 Michael Ossmann
 * Copyright 2011-2013 Dominic Spill
 *
 * This file is part of Project Ubertooth.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <string.h>

#include "bluetooth-ioa.h"


/* build info */
const char compile_info[] =
	"ubertooth " GIT_REVISION " (" COMPILE_BY "@" COMPILE_HOST ") " TIMESTAMP;


u8 hop_mode = HOP_NONE;
u8 do_hop = 0;                              // set by timer interrupt
u8 le_hop_after = 0;                        // hop after this many 1.25 ms cycles
u8 cs_no_squelch = 0;                       // rx all packets if set
int8_t cs_threshold_req=CS_THRESHOLD_DEFAULT; // requested CS threshold in dBm
int8_t cs_threshold_cur=CS_THRESHOLD_DEFAULT; // current CS threshold in dBm
volatile u8 cs_trigger;                     // set by intr on P2.2 falling (CS)
volatile u8 keepalive_trigger;              // set by timer 1/s
volatile u32 cs_timestamp;                  // CLK100NS at time of cs_trigger
u16 hop_direct_channel = 0;                 // for hopping directly to a channel
int clock_trim = 0;                         // to counteract clock drift
u32 idle_buf_clkn_high;
u32 active_buf_clkn_high;
u32 idle_buf_clk100ns;
u32 active_buf_clk100ns;
u32 idle_buf_channel = 0;
u32 active_buf_channel = 0;
u8 slave_mac_address[6] = { 0, };

/* DMA buffers */
u8 rxbuf1[DMA_SIZE];
u8 rxbuf2[DMA_SIZE];

/*
 * The active buffer is the one with an active DMA transfer.
 * The idle buffer is the one we can read/write between transfers.
 */
u8 *active_rxbuf = &rxbuf1[0];
u8 *idle_rxbuf = &rxbuf2[0];

le_state_t le = {
	.access_address = 0x8e89bed6,           // advertising channel access address
	.crc_init  = 0x555555,                  // advertising channel CRCInit
	.crc_init_reversed = 0xAAAAAA,
	.crc_verify = 1,
	.connected = 0,
};

typedef void (*data_cb_t)(char *);
data_cb_t data_cb = NULL;

typedef void (*packet_cb_t)(u8 *);
packet_cb_t packet_cb = NULL;

/* Moving average (IIR) of average RSSI of packets as scaled integers (x256). */
int16_t rssi_iir[79] = {0};
#define RSSI_IIR_ALPHA 3       // 3/256 = .012

/*
 * CLK_TUNE_TIME is the duration in units of 100 ns that we reserve for tuning
 * the radio while frequency hopping.  We start the tuning process
 * CLK_TUNE_TIME * 100 ns prior to the start of an upcoming time slot.
*/
#define CLK_TUNE_TIME   2250

/* Unpacked symbol buffers (two rxbufs) */
char unpacked[DMA_SIZE*8*2];


#define MAX_PACKET_LEN 300 //bigger then i need i think

typedef struct packet_buff_t{
//	u16 xmit_delay;
	u32 aa;
	u16 header;
	u8  len;
	u8  data[MAX_PACKET_LEN];
}packet_buff_t;

//char scrap_buff[DMA_SIZE*8];
packet_buff_t PBUFFER;


volatile u8 mode = MODE_IDLE;
volatile u8 requested_mode = MODE_IDLE;
volatile u8 modulation = MOD_BT_BASIC_RATE;
volatile u16 channel = 2441;
volatile u16 requested_channel = 0;
volatile u16 low_freq = 2400;
volatile u16 high_freq = 2483;
volatile int8_t rssi_threshold = -30;  // -54dBm - 30 = -84dBm

/* DMA linked list items */
typedef struct {
	u32 src;
	u32 dest;
	u32 next_lli;
	u32 control;
} dma_lli;

dma_lli rx_dma_lli1;
dma_lli rx_dma_lli2;

/* rx terminal count and error interrupt counters */
volatile u32 rx_tc;
volatile u32 rx_err;

/* number of rx USB packets to send */
volatile u32 rx_pkts = 0;

/* status information byte */
volatile u8 status = 0;

#define DMA_OVERFLOW  0x01
#define DMA_ERROR     0x02
#define FIFO_OVERFLOW 0x04
#define CS_TRIGGER    0x08
#define RSSI_TRIGGER  0x10



static int vendor_request_handler(u8 request, u16 *request_params, u8 *data, int *data_len)
{
	u32 command[5];
	u32 result[5];
	u64 ac_copy;
	int i; // loop counter
	u32 clock;
	int clock_offset;
	u8 length; // string length
	usb_pkt_rx *p = NULL;
	u16 reg_val;
	packet_buff_t *pbuff;

	switch (request) {
	case UBERTOOTH_BTLE_CRC_BUFFER:
		btle_calc_crc(le.crc_init_reversed, &PBUFFER.data, PBUFFER.len -3);
		*data_len = 0; 
		break;

	case UBERTOOTH_SET_BUFFER: //hack of the moment for  testing  my code add in
		memcpy(&PBUFFER, data, data_len > sizeof(PBUFFER)?sizeof(PBUFFER):data_len);		
		*data_len = 0; 
		break;

	case UBERTOOTH_GET_BUFFER: //hack of the moment for  testing  my code add ins
		memcpy(data,&PBUFFER,sizeof(packet_buff_t)); 
		*data_len = sizeof(packet_buff_t);
		break;
	
	case UBERTOOTH_TX_BUFFER:
		*data_len = 0;
		break;

	case UBERTOOTH_TX_BUFFER_LE:
		le_transmit(PBUFFER.aa, PBUFFER.len, &PBUFFER.data);
//i know its not this easy, but its a start
		*data_len = 0;
		break;

	case UBERTOOTH_PING:
		*data_len = 0;
		break;

	case UBERTOOTH_RX_SYMBOLS:
		requested_mode = MODE_RX_SYMBOLS;
		rx_pkts += request_params[0];
		if (rx_pkts == 0)
			rx_pkts = 0xFFFFFFFF;
		*data_len = 0;
		break;

	case UBERTOOTH_GET_USRLED:
		data[0] = (USRLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_USRLED:
		if (request_params[0])
			USRLED_SET;
		else
			USRLED_CLR;
		break;

	case UBERTOOTH_GET_RXLED:
		data[0] = (RXLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_RXLED:
		if (request_params[0])
			RXLED_SET;
		else
			RXLED_CLR;
		break;

	case UBERTOOTH_GET_TXLED:
		data[0] = (TXLED) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_TXLED:
		if (request_params[0])
			TXLED_SET;
		else
			TXLED_CLR;
		break;

	case UBERTOOTH_GET_1V8:
		data[0] = (CC1V8) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_1V8:
		if (request_params[0])
			CC1V8_SET;
		else
			CC1V8_CLR;
		break;

	case UBERTOOTH_GET_PARTNUM:
		command[0] = 54; /* read part number */
		iap_entry(command, result);
		data[0] = result[0] & 0xFF; /* status */
		data[1] = result[1] & 0xFF;
		data[2] = (result[1] >> 8) & 0xFF;
		data[3] = (result[1] >> 16) & 0xFF;
		data[4] = (result[1] >> 24) & 0xFF;
		*data_len = 5;
		break;

	case UBERTOOTH_RESET:
		requested_mode = MODE_RESET;
		break;

	case UBERTOOTH_GET_SERIAL:
		command[0] = 58; /* read device serial number */
		iap_entry(command, result);
		data[0] = result[0] & 0xFF; /* status */
		data[1] = result[1] & 0xFF;
		data[2] = (result[1] >> 8) & 0xFF;
		data[3] = (result[1] >> 16) & 0xFF;
		data[4] = (result[1] >> 24) & 0xFF;
		data[5] = result[2] & 0xFF;
		data[6] = (result[2] >> 8) & 0xFF;
		data[7] = (result[2] >> 16) & 0xFF;
		data[8] = (result[2] >> 24) & 0xFF;
		data[9] = result[3] & 0xFF;
		data[10] = (result[3] >> 8) & 0xFF;
		data[11] = (result[3] >> 16) & 0xFF;
		data[12] = (result[3] >> 24) & 0xFF;
		data[13] = result[4] & 0xFF;
		data[14] = (result[4] >> 8) & 0xFF;
		data[15] = (result[4] >> 16) & 0xFF;
		data[16] = (result[4] >> 24) & 0xFF;
		*data_len = 17;
		break;

#ifdef UBERTOOTH_ONE
	case UBERTOOTH_GET_PAEN:
		data[0] = (PAEN) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_PAEN:
		if (request_params[0])
			PAEN_SET;
		else
			PAEN_CLR;
		break;

	case UBERTOOTH_GET_HGM:
		data[0] = (HGM) ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_HGM:
		if (request_params[0])
			HGM_SET;
		else
			HGM_CLR;
		break;
#endif

#ifdef TX_ENABLE
	case UBERTOOTH_TX_TEST:
		requested_mode = MODE_TX_TEST;
		break;

	case UBERTOOTH_GET_PALEVEL:
		data[0] = cc2400_get(FREND) & 0x7;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_PALEVEL:
		if( request_params[0] < 8 ) {
			cc2400_set(FREND, 8 | request_params[0]);
		} else {
			return 0;
		}
		break;

	case UBERTOOTH_RANGE_TEST:
		requested_mode = MODE_RANGE_TEST;
		break;

	case UBERTOOTH_REPEATER:
		requested_mode = MODE_REPEATER;
		break;
#endif

	case UBERTOOTH_RANGE_CHECK:
		data[0] = rr.valid;
		data[1] = rr.request_pa;
		data[2] = rr.request_num;
		data[3] = rr.reply_pa;
		data[4] = rr.reply_num;
		*data_len = 5;
		break;

	case UBERTOOTH_STOP:
		requested_mode = MODE_IDLE;
		break;

	case UBERTOOTH_GET_MOD:
		data[0] = modulation;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_MOD:
		modulation = request_params[0];
		break;

	case UBERTOOTH_GET_CHANNEL:
		data[0] = channel & 0xFF;
		data[1] = (channel >> 8) & 0xFF;
		*data_len = 2;
		break;

	case UBERTOOTH_SET_CHANNEL:
		requested_channel = request_params[0];
		/* bluetooth band sweep mode, start at channel 2402 */
		if (requested_channel > MAX_FREQ) {
			hop_mode = HOP_SWEEP;
			requested_channel = 2402;
		}
		/* fixed channel mode, can be outside bluetooth band */
		else {
			hop_mode = HOP_NONE;
			requested_channel = MAX(requested_channel, MIN_FREQ);
			requested_channel = MIN(requested_channel, MAX_FREQ);
		}

		if (mode != MODE_BT_FOLLOW_LE) {
			channel = requested_channel;
			requested_channel = 0;

			/* CS threshold is mode-dependent. Update it after
			 * possible mode change. TODO - kludgy. */
			cs_threshold_calc_and_set();
		}
		break;

	case UBERTOOTH_SET_ISP:
		command[0] = 57;
		iap_entry(command, result);
		*data_len = 0; /* should never return */
		break;

	case UBERTOOTH_FLASH:
		bootloader_ctrl = DFU_MODE;
		reset();
		break;

	case UBERTOOTH_SPECAN:
		if (request_params[0] < 2049 || request_params[0] > 3072 || 
				request_params[1] < 2049 || request_params[1] > 3072 ||
				request_params[1] < request_params[0])
			return 0;
		low_freq = request_params[0];
		high_freq = request_params[1];
		requested_mode = MODE_SPECAN;
		*data_len = 0;
		break;

	case UBERTOOTH_LED_SPECAN:
		if (request_params[0] > 256)
			return 0;
		rssi_threshold = (int8_t)request_params[0];
		requested_mode = MODE_LED_SPECAN;
		*data_len = 0;
		break;

	case UBERTOOTH_GET_REV_NUM:
		data[0] = 0x00;
		data[1] = 0x00;

		length = (u8)strlen(GIT_REVISION);
		data[2] = length;

		memcpy(&data[3], GIT_REVISION, length);

		*data_len = 2 + 1 + length;
		break;

	case UBERTOOTH_GET_COMPILE_INFO:
		length = (u8)strlen(compile_info);
		data[0] = length;
		memcpy(&data[1], compile_info, length);
		*data_len = 1 + length;
		break;

	case UBERTOOTH_GET_BOARD_ID:
		data[0] = BOARD_ID;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_SQUELCH:
		cs_threshold_req = (int8_t)request_params[0];
		cs_threshold_calc_and_set();
		break;

	case UBERTOOTH_GET_SQUELCH:
		data[0] = cs_threshold_req;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_BDADDR:
		target.address = 0;
		target.access_code = 0;
		for(i=0; i < 8; i++) {
			target.address |= data[i] << 8*i;
		}
		for(i=0; i < 8; i++) {
			target.access_code |= data[i+8] << 8*i;
		}
		precalc();
		break;

	case UBERTOOTH_START_HOPPING:
		clock_offset = 0;
		for(i=0; i < 4; i++) {
			clock_offset <<= 8;
			clock_offset |= data[i];
		}
		clkn += clock_offset;
		hop_mode = HOP_BLUETOOTH;
		requested_mode = MODE_BT_FOLLOW;
		break;

	case UBERTOOTH_SET_CLOCK:
		clock = data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
		clkn = clock;
		cs_threshold_calc_and_set();
		break;

	case UBERTOOTH_SET_AFHMAP:
		for(i=0; i < 10; i++) {
			afh_map[i] = data[i];
		}
		afh_enabled = 1;
		*data_len = 10;
		precalc();
		break;

	case UBERTOOTH_CLEAR_AFHMAP:
		for(i=0; i < 10; i++) {
			afh_map[i] = 0;
		}
		afh_enabled = 0;
		*data_len = 10;
		precalc();
		break;

	case UBERTOOTH_GET_CLOCK:
		clock = clkn;
		for(i=0; i < 4; i++) {
			data[i] = (clock >> (8*i)) & 0xff;
		}
		*data_len = 4;
		break;

	case UBERTOOTH_BTLE_SNIFFING:
		rx_pkts += request_params[0];
		if (rx_pkts == 0)
			rx_pkts = 0xFFFFFFFF;
		*data_len = 0;

		do_hop = 0;
		hop_mode = HOP_BTLE;
		requested_mode = MODE_BT_FOLLOW_LE;

		queue_init();
		cs_threshold_calc_and_set();
		break;

	case UBERTOOTH_GET_ACCESS_ADDRESS:
		for(i=0; i < 4; i++) {
			data[i] = (le.access_address >> (8*i)) & 0xff;
		}
		*data_len = 4;
		break;

	case UBERTOOTH_SET_ACCESS_ADDRESS:
		le.access_address = data[0] | data[1] << 8 | data[2] << 16 | data[3] << 24;
		break;

	case UBERTOOTH_DO_SOMETHING:
		// do something! just don't commit anything here
		break;

	case UBERTOOTH_DO_SOMETHING_REPLY:
		// after you do something, tell me what you did!
		// don't commit here please
		data[0] = 0x13;
		data[1] = 0x37;
		*data_len = 2;
		break;

	case UBERTOOTH_GET_CRC_VERIFY:
		data[0] = le.crc_verify ? 1 : 0;
		*data_len = 1;
		break;

	case UBERTOOTH_SET_CRC_VERIFY:
		le.crc_verify = request_params[0] ? 1 : 0;
		break;

	case UBERTOOTH_POLL:
		p = dequeue();
		if (p != NULL) {
			memcpy(data, (void *)p, sizeof(usb_pkt_rx));
			*data_len = sizeof(usb_pkt_rx);
		} else {
			data[0] = 0;
			*data_len = 1;
		}
		break;

	case UBERTOOTH_BTLE_PROMISC:
		*data_len = 0;

		hop_mode = HOP_NONE;
		requested_mode = MODE_BT_PROMISC_LE;

		queue_init();
		cs_threshold_calc_and_set();
		break;

	case UBERTOOTH_READ_REGISTER:
		reg_val = cc2400_get(request_params[0]);
		data[0] = (reg_val >> 8) & 0xff;
		data[1] = reg_val & 0xff;
		*data_len = 2;
		break;

	case UBERTOOTH_BTLE_SLAVE:
		memcpy(slave_mac_address, data, 6);
		requested_mode = MODE_BT_SLAVE_LE;
		break;

	default:
		return 0;
	}
	return 1;
}

static void clkn_init()
{
	/*
	 * Because these are reset defaults, we're assuming TIMER0 is powered on
	 * and in timer mode.  The TIMER0 peripheral clock should have been set by
	 * clock_start().
	 */

	/* stop and reset the timer to zero */
	T0TCR = TCR_Counter_Reset;
	clkn = 0;

#ifdef TC13BADGE
	/*
	 * The peripheral clock has a period of 33.3ns.  3 pclk periods makes one
	 * CLK100NS period (100 ns).
	 */
	T0PR = 2;
#else
	/*
	 * The peripheral clock has a period of 20ns.  5 pclk periods
	 * makes one CLK100NS period (100 ns).
	 */
	T0PR = 4;
#endif
	/* 3125 * 100 ns = 312.5 us, the Bluetooth clock (CLKN). */
	T0MR0 = 3124;
	T0MCR = TMCR_MR0R | TMCR_MR0I;
	ISER0 = ISER0_ISE_TIMER0;

	/* start timer */
	T0TCR = TCR_Counter_Enable;
}

/* Update CLKN. */
void TIMER0_IRQHandler()
{
	// Use non-volatile working register to shave off a couple instructions
	u32 next;

	if (T0IR & TIR_MR0_Interrupt) {

		clkn++;
		next = clkn;

		/* Trigger hop based on mode */

		/* NONE or SWEEP -> 25 Hz */
		if (hop_mode == HOP_NONE || hop_mode == HOP_SWEEP) {
			if ((next & 0x7f) == 0)
				do_hop = 1;
		}
		/* BLUETOOTH -> 1600 Hz */
		else if (hop_mode == HOP_BLUETOOTH) {
			if ((next & 0x1) == 0)
				do_hop = 1;
		}
		/* BLUETOOTH Low Energy -> 7.5ms - 4.0s in multiples of 1.25 ms */
		else if (hop_mode == HOP_BTLE) {
			if ((next & 0x3) == 0) {
				if (le_hop_after > 0 && --le_hop_after == 0) {
					do_hop = 1;
					le_hop_after = le.hop_interval;
				}
			}
		}

		/* Keepalive trigger fires at 3200/2^9 = 6.25 Hz */
		if ((next & 0x1ff) == 0)
			keepalive_trigger = 1;

		/* Ack interrupt */
		T0MR0 = 3124 - clock_trim;
		clock_trim = 0;
		T0IR = TIR_MR0_Interrupt;
	}
}

/* EINT3 handler is also defined in ubertooth.c for TC13BADGE. */
#ifndef TC13BADGE
//static volatile u8 txledstate = 1;
void EINT3_IRQHandler()
{
	/* TODO - check specific source of shared interrupt */
	IO2IntClr = PIN_GIO6;            // clear interrupt
	cs_trigger = 1;                  // signal trigger
	cs_timestamp = CLK100NS;         // time at trigger
}
#endif // TC13BADGE

/* Sleep (busy wait) for 'millis' milliseconds. The 'wait' routines in
 * ubertooth.c are matched to the clock setup at boot time and can not
 * be used while the board is running at 100MHz. */
static void msleep(uint32_t millis)
{
	uint32_t stop_at = clkn + millis * 3125 / 1000;  // millis -> clkn ticks
	do { } while (clkn < stop_at);                   // TODO: handle wrapping
}



/* TODO - return whether hop happened, or should caller have to keep
 * track of this? */
void hop(void)
{
	do_hop = 0;

	// No hopping, if channel is set correctly, do nothing
	if (hop_mode == HOP_NONE) {
		if (cc2400_get(FSDIV) == (channel - 1))
			return;
	}

	// Slow sweep (100 hops/sec)
	else if (hop_mode == HOP_SWEEP) {
		channel += 32;
		if (channel > 2480)
			channel -= 79;
	}

	else if (hop_mode == HOP_BLUETOOTH) {
		TXLED_SET;
		channel = next_hop(clkn);
	}

	else if (hop_mode == HOP_BTLE) {
		TXLED_SET;
		channel = btle_next_hop(&le);
	}

	else if (hop_mode == HOP_DIRECT) {
		TXLED_SET;
		channel = hop_direct_channel;
	}

        /* IDLE mode, but leave amp on, so don't call cc2400_idle(). */
	cc2400_strobe(SRFOFF);
	while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

	/* Retune */
	cc2400_set(FSDIV, channel - 1);
	
	/* Update CS register if hopping.  */
	if (hop_mode > 0) {
		cs_threshold_calc_and_set();
	}

	/* Wait for lock */
	cc2400_strobe(SFSON);
	while (!(cc2400_status() & FS_LOCK));
	
	/* RX mode */
	cc2400_strobe(SRX);

}



/**
 * Called when we recieve a packet in connection following mode.
 */
void connection_follow_cb(u8 *packet) {
	int i;
	int type;

	if (le.connected) {
		// hop (8 * 1.25) = 10 ms after we see a packet on this channel
		if (le_hop_after == 0)
			le_hop_after = le.hop_interval / 2;
	}

	type = packet[4] & 0xf;

	// connect packet
	if (!le.connected && type == 0x05) {
		le.connected = 1;
		le.crc_verify = 0; // we will drop many packets if we attempt to filter by CRC

		le.access_address = 0;
		for (i = 0; i < 4; ++i)
			le.access_address |= packet[18+i] << (i*8);

#define CRC_INIT (2+4+6+6+4)
		le.crc_init = (packet[CRC_INIT+2] << 16)
					| (packet[CRC_INIT+1] << 8)
					|  packet[CRC_INIT+0];
		le.crc_init_reversed = 0;
		for (i = 0; i < 24; ++i)
			le.crc_init_reversed |= ((le.crc_init >> i) & 1) << (23 - i);

#define WIN_OFFSET (2+4+6+6+4+3+1)
		// le_hop_after = idle_rxbuf[WIN_OFFSET]-2;
		do_hop = 1;

#define HOP_INTERVAL (2+4+6+6+4+3+1+2)
		le.hop_interval = idle_rxbuf[HOP_INTERVAL];

#define HOP (2+4+6+6+4+3+1+2+2+2+2+5)
		le.hop_increment = packet[HOP] & 0x1f;
		le.channel_idx = le.hop_increment;
	}
}

#define DIVIDE_ROUND(N, D) ((N) + (D)/2) / (D)

void promisc_recover_hop_increment(u8 *packet) {
	static u32 first_ts = 0;
	if (channel == 2404) {
		first_ts = CLK100NS;
		hop_direct_channel = 2406;
		do_hop = 1;
	} else if (channel == 2406) {
		u32 second_ts = CLK100NS;
		// number of channels hopped between previous and current
		u32 channels_hopped = DIVIDE_ROUND(second_ts - first_ts,
										   le.hop_interval * LE_BASECLK);
		if (channels_hopped < 37) {
			// get the hop amount based on the number of channels hopped
			le.hop_increment = hop_interval_lut[channels_hopped];
			le.channel_idx = 1;
			le_hop_after = 0;
			le.connected = 1;

			// move on to regular connection following!
			hop_mode = HOP_BTLE;
			do_hop = 0;
			le.crc_verify = 0;
			packet_cb = connection_follow_cb;

			return;
		}
		hop_direct_channel = 2404;
		do_hop = 1;
	}
	else {
		hop_direct_channel = 2404;
		do_hop = 1;
	}
}

void promisc_recover_hop_interval(u8 *packet) {
	static u32 prev_clk = 0;
	static u32 smallest_interval = 0xffffffff;
	static int consec = 0;

	u32 cur_clk = CLK100NS;
	u32 clk_diff = cur_clk - prev_clk;
	u32 obsv_hop_interval; // observed hop interval

	// probably consecutive data packets on the same channel
	if (clk_diff < 2 * LE_BASECLK)
		return;

	if (clk_diff < smallest_interval)
		smallest_interval = clk_diff;

	obsv_hop_interval = DIVIDE_ROUND(smallest_interval, 37 * LE_BASECLK);

	if (le.hop_interval == obsv_hop_interval) {
		// 5 consecutive hop intervals: consider it legit and move on
		++consec;
		if (consec == 5) {
			packet_cb = promisc_recover_hop_increment;
			hop_direct_channel = 2404;
			hop_mode = HOP_DIRECT;
			do_hop = 1;
		}
	} else {
		le.hop_interval = obsv_hop_interval;
		consec = 0;
	}

	prev_clk = cur_clk;
}

void promisc_follow_cb(u8 *packet) {
	int i;

	// get the CRCInit
	if (!le.crc_verify && packet[4] == 0x01 && packet[5] == 0x00) {
		u32 crc = (packet[8] << 16) | (packet[7] << 8) | packet[6];

		le.crc_init = btle_reverse_crc(crc, packet + 4, 2);
		le.crc_init_reversed = 0;
		for (i = 0; i < 24; ++i)
			le.crc_init_reversed |= ((le.crc_init >> i) & 1) << (23 - i);

		le.crc_verify = 1;
		packet_cb = promisc_recover_hop_interval;
	}
}

// LFU cache of recently seen AA's
struct active_aa {
	u32 aa;
	int freq;
} active_aa_list[32] = { { 0, 0, }, };
#define AA_LIST_SIZE (int)(sizeof(active_aa_list) / sizeof(struct active_aa))

// called when we see an AA, add it to the list
void see_aa(u32 aa) {
	int i, max = -1, killme = -1;
	for (i = 0; i < AA_LIST_SIZE; ++i)
		if (active_aa_list[i].aa == aa) {
			++active_aa_list[i].freq;
			return;
		}

	// evict someone
	for (i = 0; i < AA_LIST_SIZE; ++i)
		if (active_aa_list[i].freq < max || max < 0) {
			killme = i;
			max = active_aa_list[i].freq;
		}

	active_aa_list[killme].aa = aa;
	active_aa_list[killme].freq = 1;
}


int main()
{
	ubertooth_init();
	clkn_init();
	ubertooth_usb_init(vendor_request_handler);

	memset(&PBUFFER,0x00,sizeof(packet_buff_t));

	while (1) {
		handle_usb(clkn);
		if(requested_mode != mode)
			switch (requested_mode) {
				 case MODE_RESET:
					/* Allow time for the USB command to return correctly */
					wait(1);
					reset();
					break;
				case MODE_RX_SYMBOLS:
					if (rx_pkts)
						bt_stream_rx();
					break;
				case MODE_BT_FOLLOW:
					bt_follow();
					break;
				case MODE_BT_FOLLOW_LE:
					bt_follow_le();
					break;
				case MODE_BT_PROMISC_LE:
					bt_promisc_le();
					break;
				case MODE_BT_SLAVE_LE:
					bt_slave_le();
					break;
				case MODE_TX_TEST:
					mode = MODE_TX_TEST;
					cc2400_txtest(&modulation, &channel);
					break;
				case MODE_RANGE_TEST:
					mode = MODE_RANGE_TEST;
					cc2400_rangetest(&channel);
					mode = MODE_IDLE;
					if (requested_mode == MODE_RANGE_TEST)
						requested_mode = MODE_IDLE;
					break;
				case MODE_REPEATER:
					mode = MODE_REPEATER;
					cc2400_repeater(&channel);
					break;
				case MODE_SPECAN:
					specan();
					break;
				case MODE_LED_SPECAN:
					led_specan();
					break;
				case MODE_IDLE:
					cc2400_idle();
					break;
				case MODE_LE_INJECT: //might not be needed
					break;
				default:
					/* This is really an error state, but what can you do? */
					break;
			}
	}
}
