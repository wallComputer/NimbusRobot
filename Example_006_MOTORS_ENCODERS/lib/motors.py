import math
from machine import Pin, PWM
from nrpd import *
from array import array
import rp2


def make_isr(pos):
    vals = array("i", (1, -1, 0))  # vals[2] is previous x

    @micropython.viper
    def isr(sm):
        i = ptr32(pos)  # Position
        g = ptr32(vals)
        while sm.rx_fifo():
            v: int = int(sm.get()) & 3
            i[0] += g[int((v >> 1) ^ g[2])]
            g[2] = v & 1

    return isr


class Encoder:
    def __init__(self, sm_no, base_pin, flip=False):
        self._encoderCount = 0
        self._flip = -1 if flip else 1
        self._pos = array("i", (0,))  # [pos]
        self.sm = rp2.StateMachine(sm_no, self.pio_quadrature, in_base=base_pin)
        self.sm.irq(make_isr(self._pos))  # Instantiate the closure
        self.sm.exec("set(y, 99)")  # Initialise y: guarantee different to the input
        self.sm.active(1)

    @rp2.asm_pio()
    def pio_quadrature(in_init=rp2.PIO.IN_LOW):
        wrap_target()
        label("again")
        in_(pins, 2)
        mov(x, isr)
        jmp(x_not_y, "push_data")
        mov(isr, null)
        jmp("again")
        label("push_data")
        push()
        irq(block, rel(0))
        mov(y, x)
        wrap()

    @property
    def encoderCount(self):
        self._encoderCount = self._pos[0] * self._flip
        return self._encoderCount


class motorControllers:

    def __init__(self, motorFreq=100, wheelDiameter_m=0.037, encoderPips=12, motorGearRatio=110):
        self._MSP = Pin(MOTOR_SWITCH_PIN, Pin.OUT)
        self._MLT = 0
        self.__MLTSpeed = PWM(Pin(MLTIN1))
        self.__MLTSpeed.freq(motorFreq)
        self.__MLTSpeed.duty_u16(65535)
        self.__MLTDirection = Pin(MLTIN2, Pin.OUT)
        self.__MLTDirection.value(1)
        self._MLB = 0
        self.__MLBSpeed = PWM(Pin(MLBIN1))
        self.__MLBSpeed.freq(motorFreq)
        self.__MLBSpeed.duty_u16(65535)
        self.__MLBDirection = Pin(MLBIN2, Pin.OUT)
        self.__MLBDirection.value(1)
        self._MRT = 0
        self.__MRTSpeed = PWM(Pin(MRTIN1))
        self.__MRTSpeed.freq(motorFreq)
        self.__MRTSpeed.duty_u16(65535)
        self.__MRTDirection = Pin(MRTIN2, Pin.OUT)
        self.__MRTDirection.value(1)
        self._MRB = 0
        self.__MRBSpeed = PWM(Pin(MRBIN1))
        self.__MRBSpeed.freq(motorFreq)
        self.__MRBSpeed.duty_u16(65535)
        self.__MRBDirection = Pin(MRBIN2, Pin.OUT)
        self.__MRBDirection.value(1)
        self._MA = [0, 0, 0, 0]  # Motor All Speed

        self._encoderMLT = Encoder(0, Pin(ELTA), flip=True)
        self._encoderMLB = Encoder(1, Pin(ELBA), flip=True)
        self._encoderMRT = Encoder(2, Pin(ERTA), flip=False)
        self._encoderMRB = Encoder(3, Pin(ERBB), flip=True)
        self._ELTCActual = 0
        self._ELBCActual = 0
        self._ERTCActual = 0
        self._ERBCActual = 0
        self._ELTCSet = 0
        self._ELBCSet = 0
        self._ERTCSet = 0
        self._ERBCSet = 0

        self.__WC_m = math.pi * wheelDiameter_m  # Wheel Circumference in m
        self.__ECPR = encoderPips * motorGearRatio
        self._WRBDActual = 0
        self._WRTDActual = 0
        self._WLBDActual = 0
        self._WLTDActual = 0

    @property
    def ELTMActual(self):
        return self._encoderMLT.encoderCount

    @property
    def ELBMActual(self):
        return self._encoderMLB.encoderCount

    @property
    def ERTMActual(self):
        return self._encoderMRT.encoderCount

    @property
    def ERBMActual(self):
        return self._encoderMRB.encoderCount

    @property
    def ELTMSet(self):
        return self._ELTCSet

    @ELTMSet.setter
    def ELTMSet(self, encoderCount):
        self._ELTCSet = encoderCount

    @property
    def ELBMSet(self):
        return self._ELBCSet

    @ELBMSet.setter
    def ELBMSet(self, encoderCount):
        self._ELBCSet = encoderCount

    @property
    def ERTMSet(self):
        return self._ERTCSet

    @ERTMSet.setter
    def ERTMSet(self, encoderCount):
        self._ERTCSet = encoderCount

    @property
    def ERBMSet(self):
        return self._ERBCSet

    @ERBMSet.setter
    def ERBMSet(self, encoderCount):
        self._ERBCSet = encoderCount

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
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
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
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
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
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
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
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
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

    @property
    def WLTDActual(self):
        self._WLTDActual = self._encoderMLT.encoderCount * self.__WC_m / self.__ECPR
        return self._WLTDActual

    @property
    def WLBDActual(self):
        self._WLBDActual = self._encoderMLB.encoderCount * self.__WC_m / self.__ECPR
        return self._WLBDActual

    @property
    def WRTDActual(self):
        self._WRTDActual = self._encoderMRT.encoderCount * self.__WC_m / self.__ECPR
        return self._WRTDActual

    @property
    def WRBDActual(self):
        self._WRBDActual = self._encoderMRB.encoderCount * self.__WC_m / self.__ECPR
        return self._WRBDActual
