static void cc2400_idle()
{
        cc2400_strobe(SRFOFF);
        while ((cc2400_status() & FS_LOCK)); // need to wait for unlock?

#ifdef UBERTOOTH_ONE
        PAEN_CLR;
        HGM_CLR;
#endif

        RXLED_CLR;
        TXLED_CLR;
        USRLED_CLR;
        mode = MODE_IDLE;
}

/* start un-buffered rx */
static void cc2400_rx()
{
        u16 mdmctrl;
        if (modulation == MOD_BT_BASIC_RATE) {
                mdmctrl = 0x0029; // 160 kHz frequency deviation
        } else if (modulation == MOD_BT_LOW_ENERGY) {
                mdmctrl = 0x0040; // 250 kHz frequency deviation
        } else {
                /* oops */
                return;
        }

        cc2400_set(LMTST,   0x2b22);
        cc2400_set(MDMTST0, 0x134b); // without PRNG
        cc2400_set(GRMDM,   0x0101); // un-buffered mode, GFSK
        // 0 00 00 0 010 00 0 00 0 1
        //      |  | |   |  +--------> CRC off
        //      |  | |   +-----------> sync word: 8 MSB bits of SYNC_WORD
        //      |  | +---------------> 2 preamble bytes of 01010101
        //      |  +-----------------> not packet mode
        //      +--------------------> un-buffered mode
        cc2400_set(FSDIV,   channel - 1); // 1 MHz IF
        cc2400_set(MDMCTRL, mdmctrl);

        // Set up CS register
        cs_threshold_calc_and_set();

        while (!(cc2400_status() & XOSC16M_STABLE));
        cc2400_strobe(SFSON);
        while (!(cc2400_status() & FS_LOCK));
        cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
        PAEN_SET;
        HGM_SET;
#endif
}

/* start un-buffered rx */
static void cc2400_rx_sync(u32 sync)
{
        u16 grmdm, mdmctrl;

        if (modulation == MOD_BT_BASIC_RATE) {
                mdmctrl = 0x0029; // 160 kHz frequency deviation
                grmdm = 0x0461; // un-buffered mode, packet w/ sync word detection
                // 0 00 00 1 000 11 0 00 0 1
                //   |  |  | |   |  +--------> CRC off
                //   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
                //   |  |  | +---------------> 0 preamble bytes of 01010101
                //   |  |  +-----------------> packet mode
                //   |  +--------------------> un-buffered mode
                //   +-----------------------> sync error bits: 0

        } else if (modulation == MOD_BT_LOW_ENERGY) {
                mdmctrl = 0x0040; // 250 kHz frequency deviation
                grmdm = 0x0561; // un-buffered mode, packet w/ sync word detection
                // 0 00 00 1 010 11 0 00 0 1
                //   |  |  | |   |  +--------> CRC off
                //   |  |  | |   +-----------> sync word: 32 MSB bits of SYNC_WORD
                //   |  |  | +---------------> 2 preamble bytes of 01010101
                //   |  |  +-----------------> packet mode
                //   |  +--------------------> un-buffered mode
                //   +-----------------------> sync error bits: 0

        } else {
                /* oops */
                return;
        }

        cc2400_set(MANAND,  0x7fff);
        cc2400_set(LMTST,   0x2b22);
        cc2400_set(MDMTST0, 0x134b); // without PRNG
        cc2400_set(GRMDM,   grmdm);

        cc2400_set(SYNCL,   sync & 0xffff);
        cc2400_set(SYNCH,   (sync >> 16) & 0xffff);
        
        cc2400_set(FSDIV,   channel - 1); // 1 MHz IF
        cc2400_set(MDMCTRL, mdmctrl);

        // Set up CS register
        cs_threshold_calc_and_set();

        while (!(cc2400_status() & XOSC16M_STABLE));
        cc2400_strobe(SFSON);
        while (!(cc2400_status() & FS_LOCK));
        cc2400_strobe(SRX);
#ifdef UBERTOOTH_ONE
        PAEN_SET;
        HGM_SET;
#endif
}
