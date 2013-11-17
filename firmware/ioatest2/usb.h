#ifndef __UBERTOOTH_USB_H
#define __UBERTOOTH_USB_H

#include "ubertooth.h"
#include "usb.h"
#include "fifo.h"
#include <string.h>
#include "ubertooth.h"
#include "usb_serial.h"

typedef int (VendorRequestHandler)(u8 request, u16 *request_params, u8 *data, int *data_len);



static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData);

static void BulkOut(U8 bEP, U8 bEPStatus);
static void SendNextBulkIn(U8 bEP, BOOL fFirstPacket);
static void BulkIn(U8 bEP, U8 bEPStatus);
static void USBFrameHandler(U16 wFrame);
static void USBDevIntHandler(U8 bDevStatus);

void VCOM_init(void);
int VCOM_putchar(int c);
int VCOM_getchar(void);
void USB_IRQHandler();

BOOL usb_vendor_request_handler(TSetupPacket *pSetup, int *piLen, u8 **ppbData);
void usb_ioa_init(VendorRequestHandler *vendor_req_handler);


#endif
