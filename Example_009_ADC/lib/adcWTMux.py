from machine import Pin, ADC
from nrpd import *

class adcWTMux:
    def __init__(self):
        self.__ADC_SEL_0 = Pin(ADC_SEL_0, Pin.OUT)
        self.__ADC_SEL_1 = Pin(ADC_SEL_1, Pin.OUT)
        self.__ADC_SEL_2 = Pin(ADC_SEL_2, Pin.OUT)
        self.__ADC = ADC(Pin(ADC_IN_PIN))
        self.__currentOffsetMLT = -16.62447995907153
        self.__currentOffsetMLB = -16.61263177736352
        self.__currentOffsetMRT = -16.6010798001982
        self.__currentOffsetMRB = -16.61292798190622
        self._AValue = 0

    def readFaultRightController(self):
        self.__ADC_SEL_0.value(0)
        self.__ADC_SEL_1.value(0)
        self.__ADC_SEL_2.value(0)
        return self.__ADC.read_u16()

    def readMotorRightBottomCurrent_mA(self):
        self.__ADC_SEL_0.value(1)
        self.__ADC_SEL_1.value(0)
        self.__ADC_SEL_2.value(0)
        return max(self.__ADC.read_u16()/65535*3.3/0.68*1000+self.__currentOffsetMRB, 0)

    def readMotorRightTopCurrent_mA(self):
        self.__ADC_SEL_0.value(0)
        self.__ADC_SEL_1.value(1)
        self.__ADC_SEL_2.value(0)
        return max(self.__ADC.read_u16()/65535*3.3/0.68*1000+self.__currentOffsetMRT, 0)

    def readBatteryLevel_V(self):
        self.__ADC_SEL_0.value(1)
        self.__ADC_SEL_1.value(1)
        self.__ADC_SEL_2.value(0)
        return self.__ADC.read_u16()/65535*3.3*2

    def readMotorLeftTopCurrent_mA(self):
        self.__ADC_SEL_0.value(0)
        self.__ADC_SEL_1.value(0)
        self.__ADC_SEL_2.value(1)
        return max(self.__ADC.read_u16()/65535*3.3/0.68*1000+self.__currentOffsetMLT, 0)

    def readPowerBooster_V(self):
        self.__ADC_SEL_0.value(1)
        self.__ADC_SEL_1.value(0)
        self.__ADC_SEL_2.value(1)
        return self.__ADC.read_u16()/65535*3.3*2

    def readFaultLeftController(self):
        self.__ADC_SEL_0.value(0)
        self.__ADC_SEL_1.value(1)
        self.__ADC_SEL_2.value(1)
        return self.__ADC.read_u16()

    def readMotorLeftBottomCurrent_mA(self):
        self.__ADC_SEL_0.value(1)
        self.__ADC_SEL_1.value(1)
        self.__ADC_SEL_2.value(1)
        return max(self.__ADC.read_u16()/65535*3.3/0.68*1000+self.__currentOffsetMLB, 0)





