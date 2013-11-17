# Hey Emacs, this is a -*- makefile -*-

# Target definition for lpcusb library
LPCUSB_TARGET = LPC17xx

# Target file name (without extension).
TARGET = ubertooth_ioa

# Library paths
LPCUSB_PATH = $(LIBS_PATH)/lpcusb/target

# List C source files here. (C dependencies are automatically generated.)
SRC = $(TARGET).c \
	usb.c \
	fifo.c \
	BTApp.c \
	bt_utils.c \
	hci.c \
	l2cap_2.c \
	rfcomm.c \
	rfcomm_fcs.c \
	sdp.c \
	debug.c \
	$(LIBS_PATH)/LPC17xx_Startup.c \
	$(LIBS_PATH)/LPC17xx_Interrupts.c \
	$(LIBS_PATH)/ubertooth.c \
	$(LPCUSB_PATH)/usbcontrol.c \
	$(LPCUSB_PATH)/usbinit.c \
	$(LPCUSB_PATH)/usbhw_lpc.c \
	$(LPCUSB_PATH)/usbstdreq.c

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS = $(LIBS_PATH) $(LPCUSB_PATH)

# Place -D or -U options here for C sources
CDEFS  = -D$(BOARD) -D$(LPCUSB_TARGET) $(COMPILE_OPTS)

include ../common.mk
