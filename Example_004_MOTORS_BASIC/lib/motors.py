from machine import Pin,PWM
from nrpd import *
from motorProfiles import *


def mapRange(oValue, oMin, oMax, nMin, nMax):
    oSpan = oMax - oMin
    nSpan = nMax - nMin
    scale = (oValue - oMin) / oSpan
    nValue = nMin + scale * nSpan
    return nValue

class motorControllers:
    def __init__(self):
        self.MSP = Pin(MOTOR_SWITCH_PIN, Pin.OUT)

        self._MLT = 0
        self.MLTSpeed = PWM(Pin(MLTIN1))
        self.MLTSpeed.freq(MLTf)
        self.MLTSpeed.duty_u16(65535)
        self.MLTDirection = Pin(MLTIN2, Pin.OUT)
        self.MLTDirection.value(1)
        self._MLB = 0
        self.MLBSpeed = PWM(Pin(MLBIN1))
        self.MLBSpeed.freq(MLBf)
        self.MLBSpeed.duty_u16(65535)
        self.MLBDirection = Pin(MLBIN2, Pin.OUT)
        self.MLBDirection.value(1)
        self._MRT = 0
        self.MRTSpeed = PWM(Pin(MRTIN1))
        self.MRTSpeed.freq(MRTf)
        self.MRTSpeed.duty_u16(65535)
        self.MRTDirection = Pin(MRTIN2, Pin.OUT)
        self.MRTDirection.value(1)
        self._MRB = 0
        self.MRBSpeed = PWM(Pin(MRBIN1))
        self.MRBSpeed.freq(MRBf)
        self.MRBSpeed.duty_u16(65535)
        self.MRBDirection = Pin(MRBIN2, Pin.OUT)
        self.MRBDirection.value(1)

    def turnMotionOff(self):
        self.MSP.value(0)

    def turnMotionOn(self):
        self.MSP.value(1)

    @property
    def MLT(self):
        return self._MLT

    @MLT.setter
    def MLT(self, speed):
        self._MLT = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MLTFm, 100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed/100*3.3))
            dutyValue = int(speed*65535/100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MLTBm, -100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed/100*3.3))
            dutyValue = int(65535 + speed*65535/100)
            directionValue = 1
        self.MLTDirection.value(directionValue)
        self.MLTSpeed.duty_u16(dutyValue)

    @property
    def MLB(self):
        return self._MLB

    @MLB.setter
    def MLB(self, speed):
        self._MLB = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MLBFm, 100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed / 100 * 3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MLBBm, -100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed / 100 * 3.3))
            dutyValue = int(65535 + speed * 65535 / 100)
            directionValue = 1
        self.MLBDirection.value(directionValue)
        self.MLBSpeed.duty_u16(dutyValue)


    @property
    def MRT(self):
        return self._MRT

    @MRT.setter
    def MRT(self, speed):
        self._MRT = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MRTFm, 100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed/100*3.3))
            dutyValue = int(speed*65535/100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MRTBm, -100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed/100*3.3))
            dutyValue = int(65535 + speed*65535/100)
            directionValue = 1
        self.MRTDirection.value(directionValue)
        self.MRTSpeed.duty_u16(dutyValue)

    @property
    def MRB(self):
        return self._MRB

    @MRB.setter
    def MRB(self, speed):
        self._MRT = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MRBFm, 100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed / 100 * 3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MRBBm, -100)
            print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed / 100 * 3.3))
            dutyValue = int(65535 + speed * 65535 / 100)
            directionValue = 1
        self.MRBDirection.value(directionValue)
        self.MRBSpeed.duty_u16(dutyValue)

