import time

from lib import motors

print("Hello, World!")

mc = motors.motorControllers()

for speed in range(0, 100, 2):
    mc.MLT = speed
    mc.MLB = speed
    mc.MRT = speed
    mc.MRB = speed
    print("Speed: {Speed:}".format(Speed=speed))
    time.sleep(0.1)

for speed in range(100, -100, 2):
    mc.MLT = speed
    mc.MLB = speed
    mc.MRT = speed
    mc.MRB = speed
    print("Speed: {Speed:}".format(Speed=speed))
    time.sleep(0.2/4)

for speed in range(-100, 0, 2):
    mc.MLT = speed
    mc.MLB = speed
    mc.MRT = speed
    mc.MRB = speed
    print("Speed: {Speed:}".format(Speed=speed))
    time.sleep(0.1)


mc.MLT = 0
mc.MLB = 0
mc.MRT = 0
mc.MRB = 0