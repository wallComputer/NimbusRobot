from machine import Pin, PWM, Timer
import time
import rp2
from array import array


class dcMotor:
    __defaultMotorFreq = 75  # Hz
    __defaultPIDSamplingFrequency = 100  # Hz
    __minimumAllowedSpeed = -65535
    __maximumAllowedSpeed = 65535
    __defaultPIDParams = [50, 0, 6]  # Used for micro-metal gear motor for encoder count PID controller.


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
        def __init__(self,
                     sm_no: int,
                     base_pin: Pin,
                     flip: bool = False):
            """

            Create Encoder Class. Original Code taken from https://github.com/micropython/micropython/pull/6894#issuecomment-1002717393

            :param sm_no: State machine Number. Must be an integer between 0 and 7 and unique to each encoder.

            :param base_pin: First Pin of the encoder pair. Next Pin is taken as the second input to the encoder.

            :param flip: Flip the sign of encoder count. Useful if you cannot change encoder pins in hardware.
            """
            self._flip = -1 if flip else 1
            self._pos = array("i", (0,))  # [pos]
            self.sm = rp2.StateMachine(sm_no, self.pio_quadrature, in_base=base_pin)
            self.sm.irq(dcMotor.make_isr(self._pos))  # Instantiate the closure
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

        def encoderCount(self) -> int:
            """

            Returns the encoder count of the encoder.

            :return: Encoder count
            """
            return self._pos[0] * self._flip

        def resetEncoder(self):
            """

            Resets the encoder.
            :return:
            """
            self._pos[0] = 0

    class PID:
        __defaultSetPointThreshold = 50  # With 37 mm wheels, this will give ±4.4mm error.
        __defaultThresholdSize = 50

        def __init__(self,
                     PIDParams: list = None,
                     setPointThreshold: int = __defaultSetPointThreshold,
                     thresholdSize: int = __defaultThresholdSize):
            """
            PID Controller Class

            :param PIDParams: [Kp, Ki, Kd] list

            :param setPointThreshold: if average Encoder count is within the setPoint ± this value, the motor is stopped and controller is expected to be reset (by whoever calls the calculation function).

            :param thresholdSize: Size used for calculating moving average of the Encoder Count.
            """
            self._Kp = PIDParams[0]
            self._Ki = PIDParams[1]
            self._Kd = PIDParams[2]
            self._prevErr = 0
            self._errSum = 0
            self._setPoint = 0
            self._spT = setPointThreshold
            self._tC = thresholdSize
            self._mA = [0] * self._tC
            self._wP = 0
            self._avgOk = False

        def calculatePlantInput(self, sensorReading, delT):
            """

            Calculates the next plant input U based on current values.

            :param sensorReading: Input to the controller. This will be used to find error against the setPoint of the controller.
            :param delT: to be used with the Integral and Differential term.
            :return: Tuple of speed to be applied to the plant/motor and if threshold average has been reached.
            """
            err = self._setPoint - sensorReading
            delErr_dt = 0
            if delT != 0:
                delErr_dt = (err - self._prevErr) / delT
            self._prevErr = err
            self._errSum += err * delT
            appliedU = self._Kp * err + self._Ki * self._errSum + self._Kd * delErr_dt
            self._mA[self._wP % self._tC] = sensorReading
            self._wP += 1
            if self._wP >= self._tC:
                srAvg = sum(
                    self._mA) / self._tC  # This point can be made more efficient. Do not calculate sum each time. Reuse last sum.
                if self._setPoint - self._spT <= srAvg <= self._setPoint + self._spT:
                    self._avgOk = True
            return appliedU, self._avgOk

        def setPoint(self, setPoint: int = None):
            """

            Set or Get the setPoint of the controller.

            :param setPoint: Value to be made the new setPoint.
            :return: current setPoint of the controller.
            """
            if setPoint is not None:
                self._setPoint = setPoint
            return self._setPoint

        def controllerParams(self, PIDParams: [float, float, float] = None) -> [float, float, float]:
            """

            Set or Get the Controller PID Parameters.
            :param PIDParams: [Kp, Ki, Kd] values to be set.
            :return: current [Kp, Ki, Kd]
            """
            if PIDParams is not None:
                self._Kp = PIDParams[0]
                self._Ki = PIDParams[1]
                self._Kd = PIDParams[2]
            return [self._Kp, self._Ki, self._Kd]

        def resetController(self):
            """

            Resets the controller.
            :return: None
            """
            self._prevErr = 0
            self._errSum = 0
            self._mA = [0] * self._tC
            self._wP = 0
            self._avgOk = False
            self._setPoint = 0

    def _selfCallback(self, tim):
        """

        Callback function for PID controller. Since the period of this function is not strictly the sampling frequency, the function calculates its own ∆t
        If the controller mentions that the setpoint is reached and within threshold, the function turns off the motor, resets the controller and the encoder,
        as well as turns off itself as the timer callback. To restart, set the encoder set point for the motor.

        :param tim: timer for which the callback function is used.
        :return: None
        """
        currentTms = time.ticks_ms()
        currentECount = self._encoder.encoderCount()
        delT = time.ticks_diff(currentTms, self._previousECountTmsLT) / 1e3
        self._previousECountTmsLT = currentTms
        speed, state = self._PIDDistanceController.calculatePlantInput(currentECount, delT)
        if state:
            self.motorSpeed(0)
            self._PIDDistanceController.resetController()
            self._encoder.resetEncoder()
            self._timer.init(callback=None)
        else:
            self.motorSpeed(speed)

    def __init__(self, motorParams: dict):
        """

        Each MXXParam is a dictionary with keys

        - *'motorID': Motor ID from {0, 1, 2, 3}
        - *'MPINS'*: [Pin1,Pin2] of Motor. GPIO Pin from 0 to 28 for RP2040. Must be given and be unique for a motor.
        - *'motorFreq'*: Frequency of motor. Refer datasheet or manufacturer. Default is 75Hz
        - *'minMax'*: List for minimum and maximum speed of the motor. Minimum is -65535, maximum is 65535.
        - *'sm_no'*: RP2040 PIO State machine number in [0,7]. Must be unique for each Encoder and not used elsewhere. -1 if Not to use Encoder. No filtering and control values necessary then.
        - *'EncoderPin1'*: First Pin of the encoder. Pins must be successive and unique for each Motor.
        - *'flipEncoder'*: Reverse Encoder count if needed. False by default.
        - *'usePID_Distance'*: True if PID Control of Encoder Count/ Wheel distance is needed.
        - *'PIDParams_Distance'*: Distance Controller [P, I, D] values in list.
        - *'PIDSamplingFrequency'*: Sampling Frequency of the PID Controller.

        :param motorParams: Dictionary with parameters to initialise a motor.
        """
        self._motorID = motorParams['motorID']
        try:
            PinA = motorParams['MPINS'][0]
            PinB = motorParams['MPINS'][1]
        except TypeError:
            print("Must Give Input Pins!!!")
            time.sleep(5)
            return
        if 'motorFreq' not in motorParams:
            motorParams['motorFreq'] = dcMotor.__defaultMotorFreq
        self._PWMA = PWM(Pin(PinA))
        self._PWMA.freq(motorParams['motorFreq'])
        self._PWMB = PWM(Pin(PinB))
        self._PWMA.freq(motorParams['motorFreq'])
        self._PWMA.duty_u16(0)
        self._PWMB.duty_u16(0)
        self._motorSpeed = 0
        self._previousECountTmsLT = time.ticks_ms()
        if 'minMax' not in motorParams:
            motorParams['minMax'] = [-65535, 65535]
        self._minU = motorParams['minMax'][0]
        self._maxU = motorParams['minMax'][1]

        if 'sm_no' not in motorParams or motorParams['sm_no'] == -1:
            return
        sm_no = motorParams['sm_no']
        self._encoder = None
        self._encoder = dcMotor.Encoder(sm_no=sm_no,
                                        base_pin=Pin(motorParams['EncoderPin1']),
                                        flip=motorParams['flipEncoder'])
        self._PIDDistanceController = None
        self._timer = None
        self._timerPeriod = None
        if 'usePID_Distance' not in motorParams or not motorParams['usePID_Distance']:
            return
        if 'PIDParams_Distance' not in motorParams:
            motorParams['PIDParams_Distance'] = dcMotor.__defaultPIDParams
        self._PIDDistanceController = dcMotor.PID(PIDParams=motorParams['PIDParams_Distance'])
        if 'PIDSamplingFrequency' not in motorParams:
            motorParams['PIDSamplingFrequency'] = dcMotor.__defaultPIDSamplingFrequency
        self._timerPeriod = int(1000 / motorParams['PIDSamplingFrequency'])
        self._timer = Timer()
        self._timer.init(mode=Timer.PERIODIC,
                         period=self._timerPeriod,
                         callback=self._selfCallback)

    def motorSpeed(self, speed: int = None) -> int:
        """

        Sets the motor speed if speed argument is given. Otherwise, returns the current speed.
        A negative speed value indicates the motor is rotating in reverse direction.

        :param speed: Speed of the motor from [dcMotor._minU, dcMotor._maxU]
        :return: current speed of the motor.
        """
        if speed is not None:
            if speed < self._minU:
                speed = self._minU
            elif speed > self._maxU:
                speed = self._maxU
            self._motorSpeed = speed
            if speed >= 0:
                self._PWMA.duty_u16(int(speed))
                self._PWMB.duty_u16(0)
            elif speed < 0:
                self._PWMA.duty_u16(0)
                self._PWMB.duty_u16(int(-speed))

        return self._motorSpeed

    def encoderCount(self) -> int:
        """

        Returns the encoder Count of the motor.

        :return:
        """
        return self._encoder.encoderCount()

    def encoderSetPoint(self, encoderSetPoint: int = None) -> int:
        """

        Sets or Gets the encoder set Point.
        if being set, the controller and encoder are reset and the timer callback is reattached to the timer.

        :param encoderSetPoint: setPoint value.
        :return: current encoderSetPoint.
        """
        if encoderSetPoint is not None:
            self._timer.init(callback=None)
            self.motorSpeed(0)
            self._PIDDistanceController.resetController()
            self._encoder.resetEncoder()
            self._PIDDistanceController.setPoint(setPoint=encoderSetPoint)
            self._timer.init(mode=Timer.PERIODIC,
                             period=self._timerPeriod,
                             callback=self._selfCallback)
        return self._PIDDistanceController.setPoint()

    def minMax(self, minMaxList: list[int, int] = None) -> list[int, int]:
        """

        Sets or gets the minimum and maximum speed of the motor. The values must be between [-65535,65535]

        :param minMaxList: list of [min speed, max speed]
        :return: current minMax List
        """
        if minMaxList is not None:
            self._minU = max(int(minMaxList[0]), dcMotor.__minimumAllowedSpeed)
            self._maxU = min(int(minMaxList[1]), dcMotor.__maximumAllowedSpeed)
        return [self._minU, self._maxU]

    def reset(self) -> None:
        """

        Resets the motor encoder, controller, and detaches the timer callback.

        :return:
        """
        self._timer.init(callback=None)
        self.motorSpeed(0)
        self._PIDDistanceController.setPoint(0)
        self._PIDDistanceController.resetController()
        self._encoder.resetEncoder()

    def controllerParams(self, PIDParams: list[float, float, float] = None) -> list[float, float, float]:
        """

        Sets or Gets the controller [Kp, Ki, Kd] values.

        :param PIDParams: [Kp, Ki, Kd] list
        :return: current [Kp, Ki, Kd] list
        """
        if PIDParams is not None:
            self._PIDDistanceController.controllerParams(PIDParams)
        return self._PIDDistanceController.controllerParams()
