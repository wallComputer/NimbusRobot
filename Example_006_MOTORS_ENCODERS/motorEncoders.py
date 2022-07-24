import time
from lib import motors
from lib import rotary_irq_rp2
from nrpd import *



print("Hello, World!")


mc = motors.motorControllers()

while True:
    print("LTE: {0}\t\tLBE: {1}\t\tRTE: {2}\t\tRBE: {3}".format(mc.ELTMActual, mc.ELBMActual, mc.ERTMActual, mc.ERBMActual))
    time.sleep(0.02)