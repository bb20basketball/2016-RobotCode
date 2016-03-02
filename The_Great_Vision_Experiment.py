import sys
import time
from networktables import NetworkTable
import networktables

ip = "127.0.0.1"

NetworkTable.setIPAddress(ip)
NetworkTable.setClientMode()
NetworkTable.initialize()

table = NetworkTable.getTable('/GRIP/myContoursReport')
default = networktables.NumberArray()

while True:
    try:
        table.retrieveValue('centerX', default)
    except KeyError:
        pass
    else:
        
        if len(default)>0:
            vision_number=default[0]

            #For safety reasons, you can press B and it will stop the auto line up
            if vision_number > 180:
                #self.drive1.set(-.2)
                #self.drive2.set(.2)
                print "to da right"
            elif vision_number< 140:
                #self.drive1.set(.2)
                #self.drive2.set(-.2)
                print"to da left"
            else:
                print "you good"
    time.sleep(1)
