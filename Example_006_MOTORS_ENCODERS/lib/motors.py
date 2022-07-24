from machine import Pin, PWM
from nrpd import *
from motorProfiles import *
import rotary_irq_rp2


class motorControllers:

    @staticmethod
    def mapRange(oValue, oMin, oMax, nMin, nMax):
        oSpan = oMax - oMin
        nSpan = nMax - nMin
        scale = (oValue - oMin) / oSpan
        nValue = nMin + scale * nSpan
        return nValue

    def __init__(self, setEncoders=True):
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

        self.__useEncoders = setEncoders
        if setEncoders:
            self.__encoderLeftTopMotor = rotary_irq_rp2.RotaryIRQ(pin_num_clk=ELTB, pin_num_dt=ELTA,
                                                                  min_val=-2147483647, max_val=2147483647, start_val=0,
                                                                  reverse=True,
                                                                  range_mode=rotary_irq_rp2.RotaryIRQ.RANGE_BOUNDED,
                                                                  pull_up=True, half_step=False, invert=False)
            self.__encoderLeftBottomMotor = rotary_irq_rp2.RotaryIRQ(pin_num_clk=ELBA, pin_num_dt=ELBB,
                                                                     min_val=-2147483647, max_val=2147483647,
                                                                     start_val=0,
                                                                     reverse=True,
                                                                     range_mode=rotary_irq_rp2.RotaryIRQ.RANGE_BOUNDED,
                                                                     pull_up=True, half_step=True, invert=False)
            self.__encoderRightTopMotor = rotary_irq_rp2.RotaryIRQ(pin_num_clk=ERTB, pin_num_dt=ERTA,
                                                                   min_val=-2147483647, max_val=2147483647, start_val=0,
                                                                   reverse=True,
                                                                   range_mode=rotary_irq_rp2.RotaryIRQ.RANGE_BOUNDED,
                                                                   pull_up=True, half_step=True, invert=False)
            self.__encoderRightBottomMotor = rotary_irq_rp2.RotaryIRQ(pin_num_clk=ERBB, pin_num_dt=ERBA,
                                                                      min_val=-2147483647, max_val=2147483647,
                                                                      start_val=0,
                                                                      reverse=True,
                                                                      range_mode=rotary_irq_rp2.RotaryIRQ.RANGE_BOUNDED,
                                                                      pull_up=True, half_step=True, invert=False)

        self._ELTMActual = 0
        self._ELBMActual = 0
        self._ERTMActual = 0
        self._ERBMActual = 0
        self._ELTMSet = 0
        self._ELBMSet = 0
        self._ERTMSet = 0
        self._ERBMSet = 0

    def setCallBackFunctions(self,
                             leftTopMotorCallback=None,
                             leftBottomMotorCallback=None,
                             rightTopMotorCallback=None,
                             rightBottomMotorCallback=None):
        if self.__useEncoders:
            if leftTopMotorCallback is not None:
                self.__encoderLeftTopMotor.add_listener(leftTopMotorCallback)
            if leftBottomMotorCallback is not None:
                self.__encoderLeftBottomMotor.add_listener(leftBottomMotorCallback)
            if rightTopMotorCallback is not None:
                self.__encoderRightTopMotor.add_listener(rightTopMotorCallback)
            if rightBottomMotorCallback is not None:
                self.__encoderRightBottomMotor.add_listener(rightBottomMotorCallback)

    @property
    def ELTMActual(self):
        if self.__useEncoders:
            return self.__encoderLeftTopMotor.value()
        else:
            return 0

    @property
    def ELBMActual(self):
        if self.__useEncoders:
            return self.__encoderLeftBottomMotor.value()
        else:
            return 0

    @property
    def ERTMActual(self):
        if self.__useEncoders:
            return self.__encoderRightTopMotor.value()
        else:
            return 0

    @property
    def ERBMActual(self):
        if self.__useEncoders:
            return self.__encoderRightBottomMotor.value()
        else:
            return 0

    @property
    def ELTMSet(self):
        if self.__useEncoders:
            return self._ELTMSet
        else:
            return 0

    @ELTMSet.setter
    def ELTMSet(self, encoderCount):
        if self.__useEncoders:
            self._ELTMSet = encoderCount
        else:
            self._ELTMSet = 0

    @property
    def ELBMSet(self):
        if self.__useEncoders:
            return self._ELBMSet
        else:
            return 0

    @ELBMSet.setter
    def ELBMSet(self, encoderCount):
        if self.__useEncoders:
            self._ELBMSet = encoderCount
        else:
            self._ELBMSet = 0

    @property
    def ERTMSet(self):
        if self.__useEncoders:
            return self._ERTMSet
        else:
            return 0

    @ERTMSet.setter
    def ERTMSet(self, encoderCount):
        if self.__useEncoders:
            self._ERTMSet = encoderCount
        else:
            self._ERTMSet = 0

    @property
    def ERBMSet(self):
        if self.__useEncoders:
            return self._ERBMSet
        else:
            return 0

    @ERBMSet.setter
    def ERBMSet(self, encoderCount):
        if self.__useEncoders:
            return self._ERBMSet
        else:
            self.ERBMSet = 0

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
            speed = motorControllers.mapRange(speed, 0, 100, MLTFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed/100*3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = motorControllers.mapRange(speed, 0, -100, MLTBm, -100)
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
            speed = motorControllers.mapRange(speed, 0, 100, MLBFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed / 100 * 3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = motorControllers.mapRange(speed, 0, -100, MLBBm, -100)
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
            speed = motorControllers.mapRange(speed, 0, 100, MRTFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed/100*3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = motorControllers.mapRange(speed, 0, -100, MRTBm, -100)
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
            speed = motorControllers.mapRange(speed, 0, 100, MRBFm, 100)
            # print("Set Speed% = {Speed:} VDIFF={VDIFF:0.2f}".format(Speed=speed, VDIFF=speed / 100 * 3.3))
            dutyValue = int(speed * 65535 / 100)
            directionValue = 0
        elif speed < 0:
            speed = motorControllers.mapRange(speed, 0, -100, MRBBm, -100)
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
