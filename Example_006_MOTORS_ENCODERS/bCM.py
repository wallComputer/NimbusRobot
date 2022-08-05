"""

basic Controller Movement

"""

import time
from nrpd import *
from lib import motorHandler
from machine import Pin

LTParams = {
    'motorID': 0,
    'MPINS': [MLTIN1, MLTIN2],
    'motorFreq': 75,
    'sm_no': 0,
    'EncoderPin1': ELTA,
    'flipEncoder': True,
    'usePID_Distance': True,
    'PIDParams_Distance': [80, 0, 7],
    'PIDSamplingFrequency': 100
}
LBParams = {
    'motorID': 1,
    'MPINS': [MLBIN1, MLBIN2],
    'motorFreq': 75,
    'sm_no': 1,
    'EncoderPin1': ELBA,
    'flipEncoder': True,
    'usePID_Distance': True,
    'PIDParams_Distance': [50, 0, 5],
    'PIDSamplingFrequency': 100  # Hz
}
RTParams = {
    'motorID': 2,
    'MPINS': [MRTIN1, MRTIN2],
    'motorFreq': 75,
    'sm_no': 2,
    'EncoderPin1': ERTA,
    'flipEncoder': False,
    'usePID_Distance': True,
    'PIDParams_Distance': [80, 0, 5],
    'PIDSamplingFrequency': 100
}
RBParams = {
    'motorID': 3,
    'MPINS': [MRBIN1, MRBIN2],
    'motorFreq': 75,
    'sm_no': 3,
    'EncoderPin1': ERBB,
    'flipEncoder': True,
    'usePID_Distance': True,
    'PIDParams_Distance': [80, 0, 6],
    'PIDSamplingFrequency': 100
}
time.sleep(1)
t1 = time.ticks_ms()
mc = motorHandler.nimbusMotorController(MOTOR_SWITCH_PIN,
                                        MLTParams=LTParams,
                                        MLBParams=LBParams,
                                        MRTParams=RTParams,
                                        MRBParams=RBParams)
print(time.ticks_diff(time.ticks_ms(), t1))
btn = Pin(BUTTON_PIN, Pin.IN)
sp = (37*3.141592653589793)*5


def btnIRQ(pin):
    global mc
    mc.MSP(state=1-mc.MSP())


btn.irq(trigger=Pin.IRQ_FALLING, handler=btnIRQ)

throttledSpeed = int(65535*30/100)
minMax = [-throttledSpeed, throttledSpeed]

mc.MMMA(minMaxListList=[minMax]*4)

mc.MSP(state=1)
mc.MDSPA([2000, 2000, 2000, 2000])
time.sleep(0.01)
speed = mc.MSA()
while speed[0] != 0 and speed[1] != 0 and speed[2] != 0 and speed[3] != 0:
    speed = mc.MSA()
    enC = mc.MECA()
    print("ELT:{0},ELB:{1},ERT:{2},ERB:{3}".format(enC[0], enC[1], enC[2], enC[3]))
    # print("SLT:{0},SLB:{1},SRT:{2},SRB:{3}".format(speed[0], speed[1], speed[2], speed[3]))
    time.sleep(0.5)
time.sleep(2)
mc.MDSPA([-2000, -2000, -2000, -2000])
time.sleep(0.01)
speed = mc.MSA()
while speed[0] != 0 and speed[1] != 0 and speed[2] != 0 and speed[3] != 0:
    speed = mc.MSA()
    enC = mc.MECA()
    print("ELT:{0},ELB:{1},ERT:{2},ERB:{3}".format(enC[0], enC[1], enC[2], enC[3]))
    # print("SLT:{0},SLB:{1},SRT:{2},SRB:{3}".format(speed[0], speed[1], speed[2], speed[3]))
    time.sleep(0.5)

mc.MSP(state=0)
