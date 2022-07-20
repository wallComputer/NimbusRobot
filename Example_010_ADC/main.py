import time

from machine import Pin
from nrpd import *
from lib import adcWTMux, motors
mc = motors.motorControllers()
adc = adcWTMux.adcWTMux()
# N = 4000
# currentOffsets = [0,0,0,0]
# for i in range(0,N):
#     currentOffsets[0] = currentOffsets[0]+adc.readMotorLeftTopCurrent_mA()
#     currentOffsets[1] = currentOffsets[1]+adc.readMotorLeftBottomCurrent_mA()
#     currentOffsets[2] = currentOffsets[2]+adc.readMotorRightTopCurrent_mA()
#     currentOffsets[3] = currentOffsets[3]+adc.readMotorRightBottomCurrent_mA()
#     time.sleep(0.01)
#     print(i/N*100)
#
# for i in range(0,4):
#     currentOffsets[i] = currentOffsets[i]/N
#
# mc.MSP = 1
# print("[")
# for i in range(-100,101):
#     # mc.MA = [50,50,50,50]
#     mc.MA = [i,i,i,i]
#     time.sleep(0.2)
#     print([adc.readMotorLeftTopCurrent_mA(), adc.readMotorLeftBottomCurrent_mA(),  adc.readMotorRightTopCurrent_mA(),adc.readMotorRightBottomCurrent_mA()],",")
# mc.MSP = 0
# print("]")
