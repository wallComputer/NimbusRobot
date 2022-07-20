from machine import Pin, PWM
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
        self._MSP = Pin(MOTOR_SWITCH_PIN, Pin.OUT)

        self._MLT = 0
        self.__MLTSpeed = PWM(Pin(MLTIN1))
        self.__MLTSpeed.freq(MLTf)
        self.__MLTSpeed.duty_u16(65535)
        self.__MLTDirection = Pin(MLTIN2, Pin.OUT)
        self.__MLTDirection.value(1)
        self._MLB = 0
        self.__MLBSpeed = PWM(Pin(MLBIN1))
        self.__MLBSpeed.freq(MLBf)
        self.__MLBSpeed.duty_u16(65535)
        self.__MLBDirection = Pin(MLBIN2, Pin.OUT)
        self.__MLBDirection.value(1)
        self._MRT = 0
        self.__MRTSpeed = PWM(Pin(MRTIN1))
        self.__MRTSpeed.freq(MRTf)
        self.__MRTSpeed.duty_u16(65535)
        self.__MRTDirection = Pin(MRTIN2, Pin.OUT)
        self.__MRTDirection.value(1)
        self._MRB = 0
        self.__MRBSpeed = PWM(Pin(MRBIN1))
        self.__MRBSpeed.freq(MRBf)
        self.__MRBSpeed.duty_u16(65535)
        self.__MRBDirection = Pin(MRBIN2, Pin.OUT)
        self.__MRBDirection.value(1)

        self._MA = [0, 0, 0, 0]  # Motor All Speed

    def turnMotionOff(self):
        self._MSP.value(0)

    def turnMotionOn(self):
        self._MSP.value(1)

    @property
    def MSP(self):
        return self._MSP.value()

    @MSP.setter
    def MSP(self, value: int):
        self._MSP.value(value)

    @property
    def MLT(self):
        return self._MLT

    @MLT.setter
    def MLT(self, speed):
        self._MLT = speed
        self._MA[0] = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MLTFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed/100*3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MLTBm, -100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed/100*3.3))
            dutyValue = int(65535 + speed * 65535 / 100)
            directionValue = 1
        self.__MLTDirection.value(directionValue)
        self.__MLTSpeed.duty_u16(dutyValue)

    @property
    def MLB(self):
        return self._MLB

    @MLB.setter
    def MLB(self, speed):
        self._MLB = speed
        self._MA[1] = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MLBFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed / 100 * 3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MLBBm, -100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed / 100 * 3.3))
            dutyValue = int(65535 + speed * 65535 / 100)
            directionValue = 1
        self.__MLBDirection.value(directionValue)
        self.__MLBSpeed.duty_u16(dutyValue)

    @property
    def MRT(self):
        return self._MRT

    @MRT.setter
    def MRT(self, speed):
        self._MRT = speed
        self._MA[2] = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MRTFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed/100*3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MRTBm, -100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed/100*3.3))
            dutyValue = int(65535 + speed * 65535 / 100)
            directionValue = 1
        self.__MRTDirection.value(directionValue)
        self.__MRTSpeed.duty_u16(dutyValue)

    @property
    def MRB(self):
        return self._MRB

    @MRB.setter
    def MRB(self, speed):
        self._MRT = speed
        self._MA[3] = speed
        dutyValue = 65535
        directionValue = 1
        if speed > 0:
            speed = mapRange(speed, 0, 100, MRBFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed / 100 * 3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = mapRange(speed, 0, -100, MRBBm, -100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=-speed / 100 * 3.3))
            dutyValue = int(65535 + speed * 65535 / 100)
            directionValue = 1
        self.__MRBDirection.value(directionValue)
        self.__MRBSpeed.duty_u16(dutyValue)

    @property
    def MA(self):
        return self._MA

    @MA.setter
    def MA(self, speeds):
        self.MLT = speeds[0]
        self.MLB = speeds[1]
        self.MRT = speeds[2]
        self.MRB = speeds[3]
