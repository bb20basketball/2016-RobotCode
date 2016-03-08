import sys
import time
from networktables import NetworkTable
import networktables

ip = "127.0.0.1"

"""NetworkTable.setIPAddress(ip)
NetworkTable.setClientMode()   Sometimes you need Client mode, sometimes you don't
NetworkTable.initialize()"""

table = NetworkTable.getTable('/GRIP/myContoursReport')
default = networktables.NumberArray()
while True:
    try:
        table.retrieveValue('centerX', default)
    except KeyError:
        pass
    else:
        
        if len(default)>0:
            if len(default)==1:
                vision_number=default[0]
            else:
                good=default[0]
                normal=abs(default[0]-160)
                for i in default[1:]:
                    total=abs(i-160)
                    if total <= normal:
                        normal=total
                        good=i
                vision_number=good
                    

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
            print vision_number
    time.sleep(1)
