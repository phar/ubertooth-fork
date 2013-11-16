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

#include "ubertooth.h"
#include "ubertooth_usb.h"
#include "ubertooth_interface.h"
#include "bluetooth.h"
#include "bluetooth_le.h"
#include "cc2400_rangetest.h"

#define MIN(x,y)	((x)<(y)?(x):(y))
#define MAX(x,y)	((x)>(y)?(x):(y))


le_state_t le = {
	.access_address = 0x8e89bed6,           // advertising channel access address
	.crc_init  = 0x555555,                  // advertising channel CRCInit
	.crc_init_reversed = 0xAAAAAA,
	.crc_verify = 1,
	.connected = 0,
};



static void dma_init_le()
{
	/* power up GPDMA controller */
	PCONP |= PCONP_PCGPDMA;

	/* zero out channel configs and clear interrupts */
	DMACC0Config = 0;
	DMACC1Config = 0;
	DMACC2Config = 0;
	DMACC3Config = 0;
	DMACC4Config = 0;
	DMACC5Config = 0;
	DMACC6Config = 0;
	DMACC7Config = 0;
	DMACIntTCClear = 0xFF;
	DMACIntErrClr = 0xFF;

	/* enable DMA globally */
	DMACConfig = DMACConfig_E;
	while (!(DMACConfig & DMACConfig_E));

	// page 607

	/* configure DMA channel 0 */
	DMACC0SrcAddr = (u32)&(DIO_SSP_DR);
	DMACC0DestAddr = (u32)&rxbuf1[0];
	DMACC0LLI = (u32)0;
	DMACC0Control = (DMA_SIZE) |
			(1 << 12) |        /* source burst size = 4 */
			(1 << 15) |        /* destination burst size = 4 */
			(0 << 18) |        /* source width 8 bits */
			(0 << 21) |        /* destination width 8 bits */
			DMACCxControl_DI | /* destination increment */
			DMACCxControl_I;   /* terminal count interrupt enable */

	DMACC0Config =
			DIO_SSP_SRC |
			(0x2 << 11) |           /* peripheral to memory */
			DMACCxConfig_IE |       /* allow error interrupts */
			DMACCxConfig_ITC;       /* allow terminal count interrupts */

	active_buf_clkn_high = (clkn >> 20) & 0xff;
	active_buf_clk100ns = CLK100NS;
	active_buf_channel = channel;

	/* reset interrupt counters */
	rx_tc = 0;
	rx_err = 0;
}


/* generic le mode */
void bt_generic_le(u8 active_mode)
{
	u8 *tmp = NULL;
	u8 hold;
	int i, j;
	int8_t rssi, rssi_at_trigger;

	modulation = MOD_BT_LOW_ENERGY;
	mode = active_mode;

	// enable USB interrupts
	ISER0 = ISER0_ISE_USB;

	RXLED_CLR;

	queue_init();
	dio_ssp_init();
	dma_init();
	dio_ssp_start();
	cc2400_rx();

	cs_trigger_enable();

	hold = 0;

	while (requested_mode == active_mode) {
		if (requested_channel != 0) {
			cc2400_strobe(SRFOFF);
			while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

			/* Retune */
			cc2400_set(FSDIV, channel - 1);

			/* Wait for lock */
			cc2400_strobe(SFSON);
			while (!(cc2400_status() & FS_LOCK));

			/* RX mode */
			cc2400_strobe(SRX);

			requested_channel = 0;
		}

		if (do_hop) {
			hop();
		} else {
			TXLED_CLR;
		}

		RXLED_CLR;

		/* Wait for DMA. Meanwhile keep track of RSSI. */
		rssi_reset();
		rssi_at_trigger = INT8_MIN;
		while ((rx_tc == 0) && (rx_err == 0))
		{
			rssi = (int8_t)(cc2400_get(RSSI) >> 8);
			if (cs_trigger && (rssi_at_trigger == INT8_MIN)) {
				rssi = MAX(rssi,(cs_threshold_cur+54));
				rssi_at_trigger = rssi;
			}
			rssi_add(rssi);
		}

		/* Keep buffer swapping in sync with DMA. */
		if (rx_tc % 2) {
			tmp = active_rxbuf;
			active_rxbuf = idle_rxbuf;
			idle_rxbuf = tmp;
		}

		if (rx_err) {
			status |= DMA_ERROR;
		}

		/* No DMA transfer? */
		if (!rx_tc)
			goto rx_continue;

		/* Missed a DMA trasfer? */
		if (rx_tc > 1)
			status |= DMA_OVERFLOW;

		rssi_iir_update();

		/* Set squelch hold if there was either a CS trigger, squelch
		 * is disabled, or if the current rssi_max is above the same
		 * threshold. Currently, this is redundant, but allows for
		 * per-channel or other rssi triggers in the future. */
		if (cs_trigger || cs_no_squelch) {
			status |= CS_TRIGGER;
			hold = CS_HOLD_TIME;
			cs_trigger = 0;
		}

		if (rssi_max >= (cs_threshold_cur + 54)) {
			status |= RSSI_TRIGGER;
			hold = CS_HOLD_TIME;
		}

		/* Send a packet once in a while (6.25 Hz) to keep
		 * host USB reads from timing out. */
		if (keepalive_trigger) {
			if (hold == 0)
				hold = 1;
			keepalive_trigger = 0;
		}

		/* Hold expired? Ignore data. */
		if (hold == 0) {
			goto rx_continue;
		}
		hold--;

		// copy the previously unpacked symbols to the front of the buffer
		memcpy(unpacked, unpacked + DMA_SIZE*8, DMA_SIZE*8);

		// unpack the new packet to the end of the buffer
		for (i = 0; i < DMA_SIZE; ++i) {
			/* output one byte for each received symbol (0x00 or 0x01) */
			for (j = 0; j < 8; ++j) {
				unpacked[DMA_SIZE*8 + i * 8 + j] = (idle_rxbuf[i] & 0x80) >> 7;
				idle_rxbuf[i] <<= 1;
			}
		}

		data_cb(unpacked);

	rx_continue:
		rx_tc = 0;
		rx_err = 0;
	}

	dio_ssp_stop();
	cs_trigger_disable();
}

void bt_le_sync(u8 active_mode)
{
	u8 *tmp = NULL;
	u8 hold;
	int i, j;
	int8_t rssi, rssi_at_trigger;

	modulation = MOD_BT_LOW_ENERGY;
	mode = active_mode;

	// enable USB interrupts
	ISER0 = ISER0_ISE_USB;

	RXLED_CLR;

	queue_init();
	dio_ssp_init();
	dma_init_le();
	dio_ssp_start();
	cc2400_rx_sync(0x6b7d9171); // bit-reversed access address

	cs_trigger_enable();

	hold = 0;

	while (requested_mode == active_mode) {
		if (requested_channel != 0) {
			cc2400_strobe(SRFOFF);
			while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

			/* Retune */
			cc2400_set(FSDIV, channel - 1);

			/* Wait for lock */
			cc2400_strobe(SFSON);
			while (!(cc2400_status() & FS_LOCK));

			/* RX mode */
			cc2400_strobe(SRX);

			requested_channel = 0;
		}

		if (do_hop) {
			hop();
		} else {
			TXLED_CLR;
		}

		RXLED_CLR;

		/* Wait for DMA. Meanwhile keep track of RSSI. */
		rssi_reset();
		rssi_at_trigger = INT8_MIN;
		while ((rx_tc == 0) && (rx_err == 0))
		{
			rssi = (int8_t)(cc2400_get(RSSI) >> 8);
			if (cs_trigger && (rssi_at_trigger == INT8_MIN)) {
				rssi = MAX(rssi,(cs_threshold_cur+54));
				rssi_at_trigger = rssi;
			}
			rssi_add(rssi);
		}

		/* Keep buffer swapping in sync with DMA. */
		if (rx_tc % 2) {
			/*
			tmp = active_rxbuf;
			active_rxbuf = idle_rxbuf;
			idle_rxbuf = tmp;
			*/
			idle_rxbuf = rxbuf1;
		}

		if (rx_err) {
			status |= DMA_ERROR;
		}

		/* No DMA transfer? */
		if (!rx_tc)
			goto rx_continue;

		/* Missed a DMA trasfer? */
		if (rx_tc > 1)
			status |= DMA_OVERFLOW;

		rssi_iir_update();

		/* Set squelch hold if there was either a CS trigger, squelch
		 * is disabled, or if the current rssi_max is above the same
		 * threshold. Currently, this is redundant, but allows for
		 * per-channel or other rssi triggers in the future. */
		if (cs_trigger || cs_no_squelch) {
			status |= CS_TRIGGER;
			hold = CS_HOLD_TIME;
			cs_trigger = 0;
		}

		if (rssi_max >= (cs_threshold_cur + 54)) {
			status |= RSSI_TRIGGER;
			hold = CS_HOLD_TIME;
		}

		/* Send a packet once in a while (6.25 Hz) to keep
		 * host USB reads from timing out. */
		if (keepalive_trigger) {
			if (hold == 0)
				hold = 1;
			keepalive_trigger = 0;
		}

		/* Hold expired? Ignore data. */
		if (hold == 0) {
			goto rx_continue;
		}
		hold--;

		uint32_t packet[48/4+1];
		u8 *p = (u8 *)packet;
		packet[0] = le.access_address;

		const uint32_t *whit = whitening_word[btle_channel_index(channel-2402)];
		// FIXME get rid of this hardcoded 48
		for (i = 0; i < 48; i+=4) {
			uint32_t v = idle_rxbuf[i+0] << 24
					   | idle_rxbuf[i+1] << 16
				       | idle_rxbuf[i+2] << 8
					   | idle_rxbuf[i+3] << 0;
			packet[i/4+1] = rbit(v) ^ whit[i/4];
		}

		unsigned len = (p[5] & 0x3f) + 2;
		if (len > 39)
			goto rx_flush;

		if (le.crc_verify) {
			u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, p + 4, len);
			u32 wire_crc = (p[4+len+2] << 16)
						 | (p[4+len+1] << 8)
						 | (p[4+len+0] << 0);
			if (calc_crc != wire_crc) // skip packets with a bad CRC
				goto rx_flush;
		}

		enqueue((uint8_t *)packet);

	rx_flush:
		cc2400_strobe(SFSON);
		while (!(cc2400_status() & FS_LOCK));

		// flush any excess bytes from the SSP's buffer
		DIO_SSP_DMACR &= ~SSPDMACR_RXDMAE;
		while (SSP1SR & SSPSR_RNE) {
			u8 tmp = (u8)DIO_SSP_DR;
		}

		/* RX mode */
		dma_init_le();
		dio_ssp_start();
		cc2400_strobe(SRX);

	rx_continue:
		rx_tc = 0;
		rx_err = 0;
	}

	dio_ssp_stop();
	cs_trigger_disable();
}



/* low energy connection following
 * follows a known AA around */
void cb_follow_le() {
	int i, j, k;
	int idx = whitening_index[btle_channel_index(channel-2402)];

	u32 access_address = 0;
	for (i = 0; i < 31; ++i) {
		access_address >>= 1;
		access_address |= (unpacked[i] << 31);
	}

	for (i = 31; i < DMA_SIZE * 8 + 32; i++) {
		access_address >>= 1;
		access_address |= (unpacked[i] << 31);
		if (access_address == le.access_address) {
			for (j = 0; j < 46; ++j) {
				u8 byte = 0;
				for (k = 0; k < 8; k++) {
					int offset = k + (j * 8) + i - 31;
					if (offset >= DMA_SIZE*8*2) break;
					int bit = unpacked[offset];
					if (j >= 4) { // unwhiten data bytes
						bit ^= whitening[idx];
						idx = (idx + 1) % sizeof(whitening);
					}
					byte |= bit << k;
				}
				idle_rxbuf[j] = byte;
			}

			// verify CRC
			if (le.crc_verify) {
				int len		 = (idle_rxbuf[5] & 0x3f) + 2;
				u32 calc_crc = btle_crcgen_lut(le.crc_init_reversed, idle_rxbuf + 4, len);
				u32 wire_crc = (idle_rxbuf[4+len+2] << 16)
							 | (idle_rxbuf[4+len+1] << 8)
							 |  idle_rxbuf[4+len+0];
				if (calc_crc != wire_crc) // skip packets with a bad CRC
					break;
			}

			// send to PC
			enqueue(idle_rxbuf);
			RXLED_SET;
			--rx_pkts;

			packet_cb(idle_rxbuf);

			break;
		}
	}
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

void bt_follow_le() {
	// FIXME: sync mode does not yet follow connections properly
	// bt_le_sync(MODE_BT_FOLLOW_LE);

	data_cb = cb_follow_le;
	packet_cb = connection_follow_cb;
	bt_generic_le(MODE_BT_FOLLOW_LE);
}

// divide, rounding to the nearest integer: round up at 0.5.
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

/* le promiscuous mode */
void cb_le_promisc(char *unpacked) {
	int i, j, k;
	int idx;

	// empty data PDU: 01 00
	char desired[4][16] = {
		{ 1, 0, 0, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 0, 1, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 1, 0, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
		{ 1, 0, 1, 1, 0, 0, 0, 0,
		  0, 0, 0, 0, 0, 0, 0, 0, },
	};

	for (i = 0; i < 4; ++i) {
		idx = whitening_index[btle_channel_index(channel-2402)];

		// whiten the desired data
		for (j = 0; j < (int)sizeof(desired[i]); ++j) {
			desired[i][j] ^= whitening[idx];
			idx = (idx + 1) % sizeof(whitening);
		}
	}

	// then look for that bitsream in our receive buffer
	for (i = 32; i < (DMA_SIZE*8*2 - 32 - 16); i++) {
		int ok[4] = { 1, 1, 1, 1 };
		int matching = -1;

		for (j = 0; j < 4; ++j) {
			for (k = 0; k < (int)sizeof(desired[j]); ++k) {
				if (unpacked[i+k] != desired[j][k]) {
					ok[j] = 0;
					break;
				}
			}
		}

		// see if any match
		for (j = 0; j < 4; ++j) {
			if (ok[j]) {
				matching = j;
				break;
			}
		}

		// skip if no match
		if (matching < 0)
			continue;

		// found a match! unwhiten it and send it home
		idx = whitening_index[btle_channel_index(channel-2402)];
		for (j = 0; j < 4+3+3; ++j) {
			u8 byte = 0;
			for (k = 0; k < 8; k++) {
				int offset = k + (j * 8) + i - 32;
				if (offset >= DMA_SIZE*8*2) break;
				int bit = unpacked[offset];
				if (j >= 4) { // unwhiten data bytes
					bit ^= whitening[idx];
					idx = (idx + 1) % sizeof(whitening);
				}
				byte |= bit << k;
			}
			idle_rxbuf[j] = byte;
		}

		u32 aa = (idle_rxbuf[3] << 24) |
				 (idle_rxbuf[2] << 16) |
				 (idle_rxbuf[1] <<  8) |
				 (idle_rxbuf[0]);
		see_aa(aa);

		enqueue(idle_rxbuf);

	}

	// once we see an AA 5 times, start following it
	for (i = 0; i < AA_LIST_SIZE; ++i) {
		if (active_aa_list[i].freq > 3) {
			le.access_address = active_aa_list[i].aa;
			data_cb = cb_follow_le;
			packet_cb = promisc_follow_cb;
			le.crc_verify = 0;
		}
	}
}

void bt_promisc_le() {
	// jump to a random data channel and turn up the squelch
	channel = 2440;
	cs_threshold_req = -70;
	cs_threshold_calc_and_set();

	data_cb = cb_le_promisc;
	bt_generic_le(MODE_BT_PROMISC_LE);
}

void bt_slave_le() {
	u32 calc_crc;
	int i;


	u8 adv_ind[] = {
		// LL header
		0x00, 0x09,

		// advertising address
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

		// advertising data
		0x02, 0x01, 0x05,

		// CRC (calc)
		0xff, 0xff, 0xff,
	};

	u8 adv_ind_len = sizeof(adv_ind) - 3;

	// copy the user-specified mac address
	for (i = 0; i < 6; ++i)
		adv_ind[i+2] = slave_mac_address[5-i];

	calc_crc = btle_calc_crc(le.crc_init_reversed, adv_ind, adv_ind_len);
	adv_ind[adv_ind_len+0] = (calc_crc >>  0) & 0xff;
	adv_ind[adv_ind_len+1] = (calc_crc >>  8) & 0xff;
	adv_ind[adv_ind_len+2] = (calc_crc >> 16) & 0xff;

	// spam advertising packets
	while (requested_mode == MODE_BT_SLAVE_LE) {
		ICER0 = ICER0_ICE_USB;
		ICER0 = ICER0_ICE_DMA;
		le_transmit(0x8e89bed6, adv_ind_len+3, adv_ind);
		ISER0 = ISER0_ISE_USB;
		ISER0 = ISER0_ISE_DMA;
		msleep(100);
	}
}




