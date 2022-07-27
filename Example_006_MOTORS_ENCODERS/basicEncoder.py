import time
from machine import Pin
from nrpd import *
from lib import motors

mc = motors.motorControllers()


while True:
    # print("LT:{0}\tLB:{1}\tRT:{2}\tRB:{3}\t".format(mc.ELTMActual, mc.ELBMActual, mc.ERTMActual, mc.ERBMActual)) # for actual encoder count.
    print("LT:{0}\tLB:{1}\tRT:{2}\tRB:{3}\t".format(mc.WLTDActual, mc.WLBDActual, mc.WRTDActual, mc.WRBDActual)) # for wheel distance.
    time.sleep(0.01)