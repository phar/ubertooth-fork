#ifndef __UBERTOOTH_IOATEST_MAIN
#define __UBERTOOTH_IOATEST_MAIN

/*
 * CLK100NS is a free-running clock with a period of 100 ns.  It resets every
 * 2^15 * 10^5 cycles (about 5.5 minutes) - computed from clkn and timer0 (T0TC)
 *
 * clkn is the native (local) clock as defined in the Bluetooth specification.
 * It advances 3200 times per second.  Two clkn periods make a Bluetooth time
 * slot.
 */

volatile u32 clkn;                       // clkn 3200 Hz counter
#define CLK100NS (3125*(clkn & 0xfffff) + T0TC)
#define LE_BASECLK (12500)               // 1.25 ms in units of 100ns

#define HOP_NONE      0
#define HOP_SWEEP     1
#define HOP_BLUETOOTH 2
#define HOP_BTLE      3
#define HOP_DIRECT    4





#define CS_THRESHOLD_DEFAULT (uint8_t)(-120)
#define CS_HOLD_TIME  2                     // min pkts to send on trig (>=1)

#define MIN(x,y)	((x)<(y)?(x):(y))
#define MAX(x,y)	((x)>(y)?(x):(y))


#endi#ifdef UBERTOOTH_ZERO
#define ID_VENDOR 0x1D50
#define ID_PRODUCT 0x6000
#elif defined UBERTOOTH_ONE
#define ID_VENDOR 0x1D50
#define ID_PRODUCT 0x6002
#elif defined TC13BADGE
#define ID_VENDOR 0xFFFF
#define ID_PRODUCT 0x0004
#else
#define ID_VENDOR 0xFFFF
#define ID_PRODUCT 0x0004
#endif


#include "dma.h"
#include "ubertooth_usb.h"
#include "cc2400.h"
#include "bluetooth.h"
#include "bluetooth_le.h"
#include "cc2400_rangetest.h"


#define BULK_IN_EP		0x82
#define BULK_OUT_EP		0x05

#define MAX_PACKET_SIZE	64f