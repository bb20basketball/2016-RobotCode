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
total=0
yings= networktables.NumberArray()
while True:
    try:
        table.retrieveValue('centerX', default)
        table.retrieveValue('centerY', yings)
    except KeyError:
        pass
    else:
            total=0
            if len(default)>0:
                vision_number=default[0]
                ying=yings[0]
                        

                #For safety reasons, you can press B and it will stop the auto line up
                if vision_number > 180:
                    total = ((vision_number/180)-1)*.6 #to be safe
                elif vision_number< 140:
                    total = (1-((vision_number/140)))*.6
                else:
                    print "goodX"
                if ying > 50:
                    total +=.4 #to be safe
                elif ying< 20:
                    total -=.4
                else:
                    print "goodY"


                print total
    time.sleep(1)
