/*
 * Copyright 2012 Dominic Spill
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

#include "bluetooth_le.h"

extern u8 le_channel_idx;
extern u8 le_hop_amount;

u16 btle_next_hop(le_state_t *le)
{
	le->channel_idx = (le->channel_idx + le->hop_increment) % 37;
	return btle_channel_index_to_phys(le->channel_idx);
}

u32 received_data = 0;

int btle_find_access_address(u8 *idle_rxbuf)
{
	/* Looks for an AA in the stream */
	u16 count;
	u8 curr_buf;
	int i = 0;

	if (received_data == 0) {
		for (; i<8; i++) {
			received_data <<= 8;
			received_data |= idle_rxbuf[i];
		}
	}
	curr_buf = idle_rxbuf[i];

	// Search until we're 32 symbols from the end of the buffer
	for(count = 0; count < ((8 * DMA_SIZE) - 32); count++)
	{
		if (received_data == access_address)
			return count;

		if (count%8 == 0)
			curr_buf = idle_rxbuf[++i];

		received_data <<= 1;
		curr_buf <<= 1;
	}
	return -1;
}

u8 btle_channel_index(u8 channel) {
	u8 idx;
	channel /= 2;
	if (channel == 0)
		idx = 37;
	else if (channel < 12)
		idx = channel - 1;
	else if (channel == 12)
		idx = 38;
	else if (channel < 39)
		idx = channel - 2;
	else
		idx = 39;
	return idx;
}

u16 btle_channel_index_to_phys(u8 idx) {
	u16 phys;
	if (idx < 11)
		phys = 2404 + 2 * idx;
	else if (idx < 37)
		phys = 2428 + 2 * (idx - 11);
	else if (idx == 37)
		phys = 2402;
	else if (idx == 38)
		phys = 2426;
	else
		phys = 2480;
	return phys;
}

// calculate CRC
//	note 1: crc_init's bits should be in reverse order
//	note 2: output bytes are in reverse order compared to wire
//
//		example output:
//			0x6ff46e
//
//		bytes in packet will be:
//		  { 0x6e, 0xf4, 0x6f }
//
u32 btle_calc_crc(u32 crc_init, u8 *data, int len) {
	u32 state = crc_init & 0xffffff;
	u32 lfsr_mask = 0x5a6000; // 010110100110000000000000
	int i, j;

	for (i = 0; i < len; ++i) {
		u8 cur = data[i];
		for (j = 0; j < 8; ++j) {
			int next_bit = (state ^ cur) & 1;
			cur >>= 1;
			state >>= 1;
			if (next_bit) {
				state |= 1 << 23;
				state ^= lfsr_mask;
			}
		}
	}

	return state;
}

// runs the CRC in reverse to generate a CRCInit
//
//	crc should be big endian
//	the return will be big endian
//
u32 btle_reverse_crc(u32 crc, u8 *data, int len) {
	u32 state = crc;
	u32 lfsr_mask = 0xb4c000; // 101101001100000000000000
	u32 ret;
	int i, j;

	for (i = len - 1; i >= 0; --i) {
		u8 cur = data[i];
		for (j = 0; j < 8; ++j) {
			int top_bit = state >> 23;
			state = (state << 1) & 0xffffff;
			state |= top_bit ^ ((cur >> (7 - j)) & 1);
			if (top_bit)
				state ^= lfsr_mask;
		}
	}

	ret = 0;
	for (i = 0; i < 24; ++i)
		ret |= ((state >> i) & 1) << (23 - i);

	return ret;
}

static u32 btle_crc_lut[256] = {
	0x000000, 0x01b4c0, 0x036980, 0x02dd40, 0x06d300, 0x0767c0, 0x05ba80, 0x040e40,
	0x0da600, 0x0c12c0, 0x0ecf80, 0x0f7b40, 0x0b7500, 0x0ac1c0, 0x081c80, 0x09a840,
	0x1b4c00, 0x1af8c0, 0x182580, 0x199140, 0x1d9f00, 0x1c2bc0, 0x1ef680, 0x1f4240,
	0x16ea00, 0x175ec0, 0x158380, 0x143740, 0x103900, 0x118dc0, 0x135080, 0x12e440,
	0x369800, 0x372cc0, 0x35f180, 0x344540, 0x304b00, 0x31ffc0, 0x332280, 0x329640,
	0x3b3e00, 0x3a8ac0, 0x385780, 0x39e340, 0x3ded00, 0x3c59c0, 0x3e8480, 0x3f3040,
	0x2dd400, 0x2c60c0, 0x2ebd80, 0x2f0940, 0x2b0700, 0x2ab3c0, 0x286e80, 0x29da40,
	0x207200, 0x21c6c0, 0x231b80, 0x22af40, 0x26a100, 0x2715c0, 0x25c880, 0x247c40,
	0x6d3000, 0x6c84c0, 0x6e5980, 0x6fed40, 0x6be300, 0x6a57c0, 0x688a80, 0x693e40,
	0x609600, 0x6122c0, 0x63ff80, 0x624b40, 0x664500, 0x67f1c0, 0x652c80, 0x649840,
	0x767c00, 0x77c8c0, 0x751580, 0x74a140, 0x70af00, 0x711bc0, 0x73c680, 0x727240,
	0x7bda00, 0x7a6ec0, 0x78b380, 0x790740, 0x7d0900, 0x7cbdc0, 0x7e6080, 0x7fd440,
	0x5ba800, 0x5a1cc0, 0x58c180, 0x597540, 0x5d7b00, 0x5ccfc0, 0x5e1280, 0x5fa640,
	0x560e00, 0x57bac0, 0x556780, 0x54d340, 0x50dd00, 0x5169c0, 0x53b480, 0x520040,
	0x40e400, 0x4150c0, 0x438d80, 0x423940, 0x463700, 0x4783c0, 0x455e80, 0x44ea40,
	0x4d4200, 0x4cf6c0, 0x4e2b80, 0x4f9f40, 0x4b9100, 0x4a25c0, 0x48f880, 0x494c40,
	0xda6000, 0xdbd4c0, 0xd90980, 0xd8bd40, 0xdcb300, 0xdd07c0, 0xdfda80, 0xde6e40,
	0xd7c600, 0xd672c0, 0xd4af80, 0xd51b40, 0xd11500, 0xd0a1c0, 0xd27c80, 0xd3c840,
	0xc12c00, 0xc098c0, 0xc24580, 0xc3f140, 0xc7ff00, 0xc64bc0, 0xc49680, 0xc52240,
	0xcc8a00, 0xcd3ec0, 0xcfe380, 0xce5740, 0xca5900, 0xcbedc0, 0xc93080, 0xc88440,
	0xecf800, 0xed4cc0, 0xef9180, 0xee2540, 0xea2b00, 0xeb9fc0, 0xe94280, 0xe8f640,
	0xe15e00, 0xe0eac0, 0xe23780, 0xe38340, 0xe78d00, 0xe639c0, 0xe4e480, 0xe55040,
	0xf7b400, 0xf600c0, 0xf4dd80, 0xf56940, 0xf16700, 0xf0d3c0, 0xf20e80, 0xf3ba40,
	0xfa1200, 0xfba6c0, 0xf97b80, 0xf8cf40, 0xfcc100, 0xfd75c0, 0xffa880, 0xfe1c40,
	0xb75000, 0xb6e4c0, 0xb43980, 0xb58d40, 0xb18300, 0xb037c0, 0xb2ea80, 0xb35e40,
	0xbaf600, 0xbb42c0, 0xb99f80, 0xb82b40, 0xbc2500, 0xbd91c0, 0xbf4c80, 0xbef840,
	0xac1c00, 0xada8c0, 0xaf7580, 0xaec140, 0xaacf00, 0xab7bc0, 0xa9a680, 0xa81240,
	0xa1ba00, 0xa00ec0, 0xa2d380, 0xa36740, 0xa76900, 0xa6ddc0, 0xa40080, 0xa5b440,
	0x81c800, 0x807cc0, 0x82a180, 0x831540, 0x871b00, 0x86afc0, 0x847280, 0x85c640,
	0x8c6e00, 0x8ddac0, 0x8f0780, 0x8eb340, 0x8abd00, 0x8b09c0, 0x89d480, 0x886040,
	0x9a8400, 0x9b30c0, 0x99ed80, 0x985940, 0x9c5700, 0x9de3c0, 0x9f3e80, 0x9e8a40,
	0x972200, 0x9696c0, 0x944b80, 0x95ff40, 0x91f100, 0x9045c0, 0x929880, 0x932c40
};

/*
 * Calculate a BTLE CRC one byte at a time. Thanks to Dominic Spill and
 * Michael Ossmann for writing and optimizing this.
 *
 * Arguments: CRCInit, pointer to start of packet, length of packet in
 * bytes
 * */
u32 btle_crcgen_lut(u32 crc_init, u8 *data, int len) {
	u32 state;
	int i;
	u8 key;

	state = crc_init & 0xffffff;
	for (i = 0; i < len; ++i) {
		key = data[i] ^ (state & 0xff);
		state = (state >> 8) ^ btle_crc_lut[key];
	}
	return state;
}


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




