import usb.util as util
import usb.core
import struct
import sys
import time
from array import array
import numpy


UBERTOOTH_PING            = 0
UBERTOOTH_RX_SYMBOLS      = 1
UBERTOOTH_TX_SYMBOLS      = 2
UBERTOOTH_GET_USRLED      = 3
UBERTOOTH_SET_USRLED      = 4
UBERTOOTH_GET_RXLED       = 5
UBERTOOTH_SET_RXLED       = 6
UBERTOOTH_GET_TXLED       = 7
UBERTOOTH_SET_TXLED       = 8
UBERTOOTH_GET_1V8         = 9
UBERTOOTH_SET_1V8         = 10
UBERTOOTH_GET_CHANNEL     = 11
UBERTOOTH_SET_CHANNEL     = 12
UBERTOOTH_RESET           = 13
UBERTOOTH_GET_SERIAL      = 14
UBERTOOTH_GET_PARTNUM     = 15
UBERTOOTH_GET_PAEN        = 16
UBERTOOTH_SET_PAEN        = 17
UBERTOOTH_GET_HGM         = 18
UBERTOOTH_SET_HGM         = 19
UBERTOOTH_TX_TEST         = 20
UBERTOOTH_STOP            = 21
UBERTOOTH_GET_MOD         = 22
UBERTOOTH_SET_MOD         = 23
UBERTOOTH_SET_ISP         = 24
UBERTOOTH_FLASH           = 25
BOOTLOADER_FLASH          = 26
UBERTOOTH_SPECAN          = 27
UBERTOOTH_GET_PALEVEL     = 28
UBERTOOTH_SET_PALEVEL     = 29
UBERTOOTH_REPEATER        = 30
UBERTOOTH_RANGE_TEST      = 31
UBERTOOTH_RANGE_CHECK     = 32
UBERTOOTH_GET_REV_NUM     = 33
UBERTOOTH_LED_SPECAN      = 34
UBERTOOTH_GET_BOARD_ID    = 35
UBERTOOTH_SET_SQUELCH     = 36
UBERTOOTH_GET_SQUELCH     = 37
UBERTOOTH_SET_BDADDR      = 38
UBERTOOTH_START_HOPPING   = 39
UBERTOOTH_SET_CLOCK       = 40
UBERTOOTH_GET_CLOCK       = 41
UBERTOOTH_BTLE_SNIFFING   = 42
UBERTOOTH_GET_ACCESS_ADDRESS = 43
UBERTOOTH_SET_ACCESS_ADDRESS = 44
UBERTOOTH_DO_SOMETHING    = 45
UBERTOOTH_DO_SOMETHING_REPLY = 46
UBERTOOTH_GET_CRC_VERIFY  = 47
UBERTOOTH_SET_CRC_VERIFY  = 48
UBERTOOTH_POLL            = 49
UBERTOOTH_BTLE_PROMISC    = 50
UBERTOOTH_SET_AFHMAP      = 51
UBERTOOTH_CLEAR_AFHMAP    = 52
UBERTOOTH_READ_REGISTER   = 53
UBERTOOTH_BTLE_SLAVE      = 54
UBERTOOTH_GET_COMPILE_INFO = 55
UBERTOOTH_GET_BUFFER     =  56
UBERTOOTH_SET_BUFFER     =  57
UBERTOOTH_TX_BUFFER	 = 58
UBERTOOTH_TX_BUFFER_LE	 = 59
UBERTOOTH_BTLE_CRC_BUFFER = 60


        
MOD_BT_BASIC_RATE = 0
MOD_BT_LOW_ENERGY = 1
MOD_80211_FHSS    = 2


MODE_IDLE          = 0
MODE_RX_SYMBOLS    = 1
MODE_TX_SYMBOLS    = 2
MODE_TX_TEST       = 3
MODE_SPECAN        = 4
MODE_RANGE_TEST    = 5
MODE_REPEATER      = 6
MODE_LED_SPECAN    = 7
MODE_BT_FOLLOW     = 8
MODE_BT_FOLLOW_LE  = 9
MODE_BT_PROMISC_LE = 10
MODE_RESET         = 11
MODE_BT_SLAVE_LE   = 12
MODE_LE_INJECT     = 13


MAX_LEN		= 4096
    
VENDOR_OUT = usb.util.build_request_type(        # 40
                  usb.util.CTRL_OUT,                 # 00
                  usb.util.CTRL_TYPE_VENDOR,         # 40
                  usb.util.CTRL_RECIPIENT_DEVICE)    # 00
    
VENDOR_IN = usb.util.build_request_type(         # c0
                  usb.util.CTRL_IN,                  # 80
                  usb.util.CTRL_TYPE_VENDOR,         # 40
                  usb.util.CTRL_RECIPIENT_DEVICE)    # 00
    
CTRL_OUT = usb.util.build_request_type(          # 21
                  usb.util.CTRL_OUT,                 # 00
                  usb.util.CTRL_TYPE_CLASS,          # 20
                  usb.util.CTRL_RECIPIENT_INTERFACE) # 01
   
 
CTRL_IN = usb.util.build_request_type(           # a1
                  usb.util.CTRL_IN,                  # 80
                  usb.util.CTRL_TYPE_CLASS,          # 20
                  usb.util.CTRL_RECIPIENT_INTERFACE) # 00

UBERTOOTH_IDS = [
        (0x1D50, 0x6002), # UBERTOOTH_ONE
        (0x1D50, 0x6000), # UBERTOOTH_ZERO
        (0xFFFF, 0x0004)  # TC13BADGE / Ubertooth with older firmware
    ]


class Ubertooth(object):
    STATE_IDLE   = 0
    STATE_ACTIVE = 1
    
    def __init__(self, deviceid=0):
	#FIXME  this should support multiple ids!, or better yet, refer to them by serial number
    	for vendor, product in UBERTOOTH_IDS:
        	self._device = usb.core.find(idVendor=vendor, idProduct=product)
        	if self._device:
        	    break
    	else:
        	raise Exception('Device not found')
        self._device.default_timeout = 3000
        self._device.set_configuration()
	self.id = deviceid


    def ping(self):
        try:
		self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_PING, 0, 0,0)
		return 1
	except:
		return 0

    def packetbuff(self,buff=None):
	if buff == None:
	       return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_BUFFER,0,0,MAX_LEN)
	else:
	       return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_BUFFER,0,0,buff)

    def serialnum(self):
        return list(self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_SERIAL, 0, 0,MAX_LEN))

    def compileinfo(self): #is the broken in firmware?
        return list(self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_COMPILE_INFO, 0, 0,MAX_LEN))

    def boardid(self):
        return list(self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_BOARD_ID, 0, 0,MAX_LEN))

    def partnum(self):
        return list(self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_PARTNUM, 0, 0,MAX_LEN))

    def revnum(self):
        return list(self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_REV_NUM, 0, 0,MAX_LEN))

    def read_register(self,reg):
        return list(self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_READ_REGISTER, reg, 0,MAX_LEN))

    def reset(self):
	try:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_RESET,0,0)
	except: #fixme
		pass

    def txbuffer(self):

        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_TX_BUFFER,0,0)

    def txbuffer_le(self):
	self.modulation(MOD_BT_LOW_ENERGY)
        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_TX_BUFFER_LE,0,0)

    def crcbuffer_le(self):
        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_BTLE_CRC_BUFFER,0,0)

    def stop(self):
        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_STOP,0,0)


    def repeater(self):
        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_REPEATER,0,0,0)


    def btle_promisc(self):
        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_BTLE_PROMISC,0,0,0)

    def btle_slave(self,mac_addr):
	a = list("".join("aa:bb:cc:dd:ee:ff".split(":")).decode("hex"))
        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_BTLE_PROMISC,0,0,struct.pack("6c",a[0],a[1],a[2],a[3],a[4],a[5]))


    def rx_led(self,rxled=None):
	ubertooth.boardid()
	if rxled == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_RXLED,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_RXLED,rxled,0)


    def clock(self,clk=None):
	if clk == None:
	       return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_CLOCK,0,0,MAX_LEN)
	else:
	       return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_CLOCK,0,0,struct.pack(">I",clk))


    def start_hopping(self,clk_offset=None):
       return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_START_HOPPING,0,0,struct.pack(">I",clk_offset))


    def user_led(self,userled=None):
	if userled== None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_USRLED,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_USRLED,userled,0)


    def squelch(self,squelch=None):
	if squelch == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_SQUELCH,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_SQUELCH,squelch,0)


    def modulation(self,modulation=None):
	if modulation == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_MOD,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_MOD,modulation,0)


    def channel(self,channel=None):
	if channel == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_CHANNEL,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_CHANNEL, channel,0)



    def palevel(self,palevel=None):
	if palevel == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_PALEVEL,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_PALEVEL, palevel,0)

    def crcverify(self,verify=None):
	if verify == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_CRC_VERIFY,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_CRC_VERIFY, verify,0)


    def access_address(self,access_address=None):
	if access_address == None:
	        return self._device.ctrl_transfer(VENDOR_IN, UBERTOOTH_GET_ACCESS_ADDRESS,0,0,MAX_LEN)
	else:
	        return self._device.ctrl_transfer(VENDOR_OUT, UBERTOOTH_SET_ACCESS_ADDRESS, 0,0,struct.pack(">I",access_address))

    
if __name__ == '__main__':

	ubertooth = Ubertooth(0)
	print "Serial Number: %s" % ubertooth.serialnum()
	print "Board ID: %s" % ubertooth.boardid()
	print "Part Number: %s" % ubertooth.partnum()
	print "Revision Number: %s" % ubertooth.revnum()
#	print "Compile Info: %s" % ubertooth.compileinfo()
	print "well.. i guess if your reading this and not a backtrace we're doing alright"
	print ubertooth.packetbuff()
	print ubertooth.packetbuff("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
	print ubertooth.packetbuff()
#	ubertooth.reset()
