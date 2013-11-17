#ifndef __UBERTOOTH_IOATEST_DMA
#define __UBERTOOTH_IOATEST_DMA



static void dma_init();
static void dma_init_le();
void DMA_IRQHandler();

static void dio_ssp_start();
static void dio_ssp_stop();

#endif