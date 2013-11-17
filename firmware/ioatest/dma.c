static void dma_init()
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
    
	/* DMA linked lists */
	rx_dma_lli1.src = (u32)&(DIO_SSP_DR);
	rx_dma_lli1.dest = (u32)&rxbuf1[0];
	rx_dma_lli1.next_lli = (u32)&rx_dma_lli2;
	rx_dma_lli1.control = (DMA_SIZE) |
    (1 << 12) |        /* source burst size = 4 */
    (1 << 15) |        /* destination burst size = 4 */
    (0 << 18) |        /* source width 8 bits */
    (0 << 21) |        /* destination width 8 bits */
    DMACCxControl_DI | /* destination increment */
    DMACCxControl_I;   /* terminal count interrupt enable */
    
	rx_dma_lli2.src = (u32)&(DIO_SSP_DR);
	rx_dma_lli2.dest = (u32)&rxbuf2[0];
	rx_dma_lli2.next_lli = (u32)&rx_dma_lli1;
	rx_dma_lli2.control = (DMA_SIZE) |
    (1 << 12) |        /* source burst size = 4 */
    (1 << 15) |        /* destination burst size = 4 */
    (0 << 18) |        /* source width 8 bits */
    (0 << 21) |        /* destination width 8 bits */
    DMACCxControl_DI | /* destination increment */
    DMACCxControl_I;   /* terminal count interrupt enable */
    
	/* disable DMA interrupts */
	ICER0 = ICER0_ICE_DMA;
    
	/* enable DMA globally */
	DMACConfig = DMACConfig_E;
	while (!(DMACConfig & DMACConfig_E));
    
	/* configure DMA channel 1 */
	DMACC0SrcAddr = rx_dma_lli1.src;
	DMACC0DestAddr = rx_dma_lli1.dest;
	DMACC0LLI = rx_dma_lli1.next_lli;
	DMACC0Control = rx_dma_lli1.control;
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

void DMA_IRQHandler()
{
	idle_buf_clkn_high = active_buf_clkn_high;
	active_buf_clkn_high = (clkn >> 20) & 0xff;
    
	idle_buf_clk100ns = active_buf_clk100ns;
	active_buf_clk100ns = CLK100NS;
    
	idle_buf_channel = active_buf_channel;
	active_buf_channel = channel;
    
	/* interrupt on channel 0 */
	if (DMACIntStat & (1 << 0)) {
		if (DMACIntTCStat & (1 << 0)) {
			DMACIntTCClear = (1 << 0);
			++rx_tc;
		}
		if (DMACIntErrStat & (1 << 0)) {
			DMACIntErrClr = (1 << 0);
			++rx_err;
		}
	}
}

static void dio_ssp_start()
{
	/* make sure the (active low) slave select signal is not active */
	DIO_SSEL_SET;
    
	/* enable rx DMA on DIO_SSP */
	DIO_SSP_DMACR |= SSPDMACR_RXDMAE;
	DIO_SSP_CR1 |= SSPCR1_SSE;
	
	/* enable DMA */
	DMACC0Config |= DMACCxConfig_E;
	ISER0 = ISER0_ISE_DMA;
    
	/* activate slave select pin */
	DIO_SSEL_CLR;
}

static void dio_ssp_stop()
{
	; // TBD
}

