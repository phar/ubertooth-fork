import time
import Ubertooth



        
ubertooth = Ubertooth.Ubertooth(0)
        
print "Serial Number: %s" % ubertooth.serialnum()
print "Board ID: %s" % ubertooth.boardid()
print "Part Number: %s" % ubertooth.partnum()
print "Revision Number: %s" % ubertooth.revnum()
        
#       ubertooth.reset()

pkt = [0xd6,0xbe,0x89,0x8e,0x40,0x23,0x2d,0x60,0x84,0x23,0xae,0x73,0x02,0x01,0x1a,0x11,0x07,0xb0,0xc1,0xa0,0x68,0x07,0xe3,0x46,0xaa,0xe5,0x4f,0x42,0xe9,0x47,0x02,0x13,0x86,0x07,0x09,0x55,0x6e,0x69,0x4b,0x65,0x79,0xc3,0xc7,0x9e]

do_adv_index = 37;
                
if (do_adv_index == 37):
	channel = 2402
elif (do_adv_index == 38):
	channel = 2426
else:
	channel = 2480

ubertooth.channel(channel)

ubertooth.packetbuff(pkt)
ubertooth.crcbuffer_le()

while 1:
	ubertooth.txbuffer_le()
	time.sleep(.2)
	print "boop"
