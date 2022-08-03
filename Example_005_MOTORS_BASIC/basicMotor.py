import time
from lib import motorHandler
from nrpd import *

print("Hello, World!")

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
mc = motorHandler.nimbusMotorController(MOTOR_SWITCH_PIN,
                                        MLTParams=LTParams,
                                        MLBParams=LBParams,
                                        MRTParams=RTParams,
                                        MRBParams=RBParams)

mc.MSP(state=1)

for speed in range(0, 100, 2):
    mc.MLTS(int(65535 * speed / 100))
    mc.MLBS(int(65535 * speed / 100))
    mc.MRTS(int(65535 * speed / 100))
    mc.MRBS(int(65535 * speed / 100))
    print("Speed:{Speed:}".format(Speed=speed))
    time.sleep(0.1)

for speed in range(100, -100, 2):
    mc.MSA([int(65535 * speed / 100), int(65535 * speed / 100), int(65535 * speed / 100), int(65535 * speed / 100)])
    print("Speed:{Speed:}".format(Speed=speed))
    time.sleep(0.2 / 4)

for speed in range(-100, 0, 2):
    mc.MLTS(int(65535 * speed / 100))
    mc.MLBS(int(65535 * speed / 100))
    mc.MRTS(int(65535 * speed / 100))
    mc.MRBS(int(65535 * speed / 100))
    print("Speed:{Speed:}".format(Speed=speed))
    time.sleep(0.1)

mc.MSP = 0
mc.MSA([0, 0, 0, 0])
