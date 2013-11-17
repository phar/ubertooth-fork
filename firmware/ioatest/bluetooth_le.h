#ifndef __UBERTOOTH_IOATEST_LE
#define __UBERTOOTH_IOATEST_LE


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

u16 btle_next_hop(le_state_t *le);
int btle_find_access_address(u8 *idle_rxbuf);
u8 btle_channel_index(u8 channel);
u16 btle_channel_index_to_phys(u8 idx);
u32 btle_calc_crc(u32 crc_init, u8 *data, int len);
u32 btle_reverse_crc(u32 crc, u8 *data, int len);
u32 btle_crcgen_lut(u32 crc_init, u8 *data, int len);

void bt_generic_le(u8 active_mode)
void bt_le_sync(u8 active_mode)
void cb_follow_le()
void connection_follow_cb(u8 *packet)
void bt_follow_le()
void promisc_recover_hop_increment(u8 *packet);
void promisc_recover_hop_interval(u8 *packet)
void promisc_follow_cb(u8 *packet)

#define DIVIDE_ROUND(N, D) ((N) + (D)/2) / (D)
#define AA_LIST_SIZE (int)(sizeof(active_aa_list) / sizeof(struct active_aa))


// LFU cache of recently seen AA's
struct active_aa {
	u32 aa;
	int freq;
} active_aa_list[32] = { { 0, 0, }, };


void see_aa(u32 aa);
void cb_le_promisc(char *unpacked);
void bt_promisc_le();
void bt_slave_le();






#endif