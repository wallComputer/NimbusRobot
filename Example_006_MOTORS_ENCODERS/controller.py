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
# time.sleep(3)
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

# duration = 16.0
# mc.MSP = 1
# startTime = time.ticks_ms()
# while time.ticks_diff(time.ticks_ms(), startTime) / 1e3 < duration:
#     mc.MLBDSP = sp
#     while mc.MLBDSP != 0:
#         print("SP:{0},E:{1}".format(mc.MLBDSP, mc.MLBD))
#         mc.ELTC
#         time.sleep(0.05)
#     time.sleep(0.5)
#     sp = -sp
# mc.MSP = 0
