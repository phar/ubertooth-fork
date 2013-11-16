from scapy.all import *
from scapy.utils import rdpcap
import Ubertooth


a=rdpcap("kevo-token.pcap")

        
ubertooth = Ubertooth.Ubertooth(0)
        
print "Serial Number: %s" % ubertooth.serialnum()
print "Board ID: %s" % ubertooth.boardid()
print "Part Number: %s" % ubertooth.partnum()
print "Revision Number: %s" % ubertooth.revnum()
        
#       ubertooth.reset()


for i in a:
	ubertooth.packetbuff(str(i)[24:])
	print ubertooth.packetbuff()
