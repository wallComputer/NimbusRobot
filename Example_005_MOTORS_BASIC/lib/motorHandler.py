from nrpd import *
from machine import Pin, Timer
import motor
import math


class nimbusMotorController:
    __wheelDiameter = 37  # mm
    __gearRatio = 110
    __encoderPoleCount = 12
    __pi = 3.141592653589793
    __encoderCountsToDistance = __wheelDiameter * __pi / __gearRatio / __encoderPoleCount  # mm
    __distanceToEncoderCounts = 1 / __encoderCountsToDistance  # 1/mm
    __defaultTimeout = 2500
    __minimumTimeout = 100

    def _selfCallBack(self, tim):
        self._MSP.value(0)
        self._MLT.motorSpeed(speed=0)
        self._MLB.motorSpeed(speed=0)
        self._MRT.motorSpeed(speed=0)
        self._MRB.motorSpeed(speed=0)

    def __init__(self,
                 MSP: int = None,
                 MLTParams: dict = None,
                 MLBParams: dict = None,
                 MRTParams: dict = None,
                 MRBParams: dict = None,
                 timeOut: int = None):
        """

        :param MSP: Motor Switch Pin.
        :param MLTParams: Dictionary for Motor Left Top
        :param MLBParams: Dictionary for Motor Left Bottom
        :param MRTParams: Dictionary for Motor Right Top
        :param MRBParams: Dictionary for Motor Right Bottom
        """
        if MSP is not None:
            self._MSP = Pin(MSP, Pin.OUT)
            self._MSP.value(0)
        else:
            self._MSP = None

        self._MLT = motor.dcMotor(MLTParams)
        self._MLB = motor.dcMotor(MLBParams)
        self._MRT = motor.dcMotor(MRTParams)
        self._MRB = motor.dcMotor(MRBParams)
        self._timeOut = timeOut if timeOut is not None else nimbusMotorController.__defaultTimeout
        self._timer = Timer()

    def MSP(self, state: int = None) -> int:
        """

        Set or Get the Motor Switch Pin state.

        :param state: to set the Motor Switch pin. 1 to set, 0 to reset.
        :return: current state of motor switch pin.
        """
        if state is not None:
            self._MSP.value(state)
        return self._MSP.value()

    def MLTS(self, speed: int = None) -> int:
        """

        Set or Get the Motor Left Top Speed.

        :param speed: Speed value between [motor.min, motor.max]
        :return:
        """
        if speed is not None:
            self._MLT.motorSpeed(speed)
        return self._MLT.motorSpeed()

    def MLBS(self, speed: int = None) -> int:
        """

        Set or Get the Motor Left Bottom Speed.

        :param speed: Speed value between [motor.min, motor.max]
        :return:
        """
        if speed is not None:
            self._MLB.motorSpeed(speed)
        return self._MLB.motorSpeed()

    def MRTS(self, speed: int = None) -> int:
        """

        Set or Get the Motor Right Top Speed.

        :param speed: Speed value between [motor.min, motor.max]
        :return:
        """
        if speed is not None:
            self._MRT.motorSpeed(speed)
        return self._MRT.motorSpeed()

    def MRBS(self, speed: int = None) -> int:
        """

        Set or Get the Motor Right Bottom Speed.

        :param speed: Speed value between [motor.min, motor.max]
        :return:
        """
        if speed is not None:
            self._MRB.motorSpeed(speed)
        return self._MRB.motorSpeed()

    def MSA(self, speedList: list[int, int, int, int] = None) -> list[int, int, int, int]:
        """

        Sets or Gets the speed of all motors at once.

        :param speedList: [Motor Left Top Speed, Motor Left Bottom Speed, Motor Right Top Speed, Motor Right Bottom Speed]
        :return: current motor speeds as list
        """
        if speedList is not None:
            self._MLT.motorSpeed(speedList[0])
            self._MLB.motorSpeed(speedList[1])
            self._MRT.motorSpeed(speedList[2])
            self._MRB.motorSpeed(speedList[3])
        return [self._MLT.motorSpeed(), self._MLB.motorSpeed(), self._MRT.motorSpeed(), self._MRB.motorSpeed()]

    def MLTEC(self) -> int:
        """

        Returns the encoder count of Motor Left Top.

        :return:
        """
        return self._MLT.encoderCount()

    def MLBEC(self) -> int:
        """

        Returns the encoder count of Motor Left Bottom.

        :return:
        """
        return self._MLB.encoderCount()

    def MRTEC(self) -> int:
        """

        Returns the encoder count of Motor Right Top.

        :return:
        """
        return self._MRT.encoderCount()

    def MRBEC(self) -> int:
        """

        Returns the encoder count of Motor Right Bottom.

        :return:
        """
        return self._MRB.encoderCount()

    def MECA(self) -> [int, int, int, int]:
        """

        returns the current encoder count of all motors.

        :return: encoder counts in order [Left Top, Left Bottom, Right Top, Right Bottom]
        """
        return [self._MLT.encoderCount(), self._MLB.encoderCount(), self._MRT.encoderCount(), self._MRB.encoderCount()]

    def MLTESP(self, encoderSetPoint: int = None) -> int:
        """

        Set or Get the Encoder set point of the Motor Left Top.

        :param encoderSetPoint: set point of the encoder.
        :return: current encoder set point
        """
        if encoderSetPoint is not None:
            self._MLT.encoderSetPoint(encoderSetPoint=encoderSetPoint)
        return self._MLT.encoderSetPoint()

    def MLBESP(self, encoderSetPoint: int = None) -> int:
        """

        Set or Get the Encoder set point of the Motor Left Bottom.

        :param encoderSetPoint: set point of the encoder.
        :return: current encoder set point
        """
        if encoderSetPoint is not None:
            self._MLB.encoderSetPoint(encoderSetPoint=encoderSetPoint)
        return self._MLB.encoderSetPoint()

    def MRTESP(self, encoderSetPoint: int = None) -> int:
        """

        Set or Get the Encoder set point of the Motor Right Top.

        :param encoderSetPoint: set point of the encoder.
        :return: current encoder set point
        """
        if encoderSetPoint is not None:
            self._MRT.encoderSetPoint(encoderSetPoint=encoderSetPoint)
        return self._MRT.encoderSetPoint()

    def MRBESP(self, encoderSetPoint: int = None) -> int:
        """

        Set or Get the Encoder set point of the Motor Right Bottom.

        :param encoderSetPoint: set point of the encoder.
        :return: current encoder set point
        """
        if encoderSetPoint is not None:
            self._MRB.encoderSetPoint(encoderSetPoint=encoderSetPoint)
        return self._MRB.encoderSetPoint()

    def MESPA(self, encoderSetPointList: list[int, int, int, int] = None) -> list[int, int, int, int]:
        """

        Sets or gets the encoder set point for all motors at once.

        :param encoderSetPointList:  encoder counts set points in order [Left Top, Left Bottom, Right Top, Right Bottom]
        :return: encoder set points
        """
        if encoderSetPointList is not None:
            self._MLT.encoderSetPoint(encoderSetPoint=encoderSetPointList[0])
            self._MLB.encoderSetPoint(encoderSetPoint=encoderSetPointList[1])
            self._MRT.encoderSetPoint(encoderSetPoint=encoderSetPointList[2])
            self._MRB.encoderSetPoint(encoderSetPoint=encoderSetPointList[3])
        return [self._MLT.encoderSetPoint(), self._MLB.encoderSetPoint(), self._MRT.encoderSetPoint(),
                self._MRB.encoderSetPoint()]

    def MLTD(self) -> float:
        """

        returns the distance rotated by the wheel based on encoder counts of the Motor Left Top.
        :return: distance moved in mm.
        """
        return self._MLT.encoderCount() * nimbusMotorController.__encoderCountsToDistance

    def MLBD(self) -> float:
        """

        returns the distance rotated by the wheel based on encoder counts of the Motor Left Bottom.
        :return: distance moved in mm.
        """
        return self._MLB.encoderCount() * nimbusMotorController.__encoderCountsToDistance

    def MRTD(self) -> float:
        """

        returns the distance rotated by the wheel based on encoder counts of the Motor Right Top.
        :return: distance moved in mm.
        """
        return self._MRT.encoderCount() * nimbusMotorController.__encoderCountsToDistance

    def MRBD(self) -> float:
        """

        returns the distance rotated by the wheel based on encoder counts of the Motor Right Bottom.
        :return: distance moved in mm.
        """
        return self._MRB.encoderCount() * nimbusMotorController.__encoderCountsToDistance

    def MDA(self) -> [float, float, float, float]:
        """

        Get Distance moved by all motors in a list

        :return:  wheel distance in mm in order [Left Top, Left Bottom, Right Top, Right Bottom]
        """
        return [self._MLT.encoderCount() * nimbusMotorController.__encoderCountsToDistance,
                self._MLB.encoderCount() * nimbusMotorController.__encoderCountsToDistance,
                self._MRT.encoderCount() * nimbusMotorController.__encoderCountsToDistance,
                self._MRB.encoderCount() * nimbusMotorController.__encoderCountsToDistance]

    def MLTDSP(self, distanceSetPoint: float = None) -> float:
        """

        Set or Get the Motor Distance set point of the Motor Left Top.

        :param distanceSetPoint: set point of the encoder in mm.
        :return: current encoder set point in mm
        """
        if distanceSetPoint is not None:
            self._MLT.encoderSetPoint(
                encoderSetPoint=int(distanceSetPoint * nimbusMotorController.__distanceToEncoderCounts))
        return self._MLT.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance

    def MLBDSP(self, distanceSetPoint: float = None) -> float:
        """

        Set or Get the Motor Distance set point of the Motor Left Bottom.

        :param distanceSetPoint: set point of the encoder in mm.
        :return: current encoder set point in mm
        """
        if distanceSetPoint is not None:
            self._MLB.encoderSetPoint(
                encoderSetPoint=int(distanceSetPoint * nimbusMotorController.__distanceToEncoderCounts))
        return self._MLB.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance

    def MRTDSP(self, distanceSetPoint: float = None) -> float:
        """

        Set or Get the Motor Distance set point of the Motor Right Top.

        :param distanceSetPoint: set point of the encoder in mm.
        :return: current encoder set point in mm
        """
        if distanceSetPoint is not None:
            self._MRT.encoderSetPoint(
                encoderSetPoint=int(distanceSetPoint * nimbusMotorController.__distanceToEncoderCounts))
        return self._MRT.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance

    def MRBDSP(self, distanceSetPoint: float = None) -> float:
        """

        Set or Get the Motor Distance set point of the Motor Right Bottom.

        :param distanceSetPoint: set point of the encoder in mm.
        :return: current encoder set point in mm
        """
        if distanceSetPoint is not None:
            self._MRB.encoderSetPoint(
                encoderSetPoint=int(distanceSetPoint * nimbusMotorController.__distanceToEncoderCounts))
        return self._MRB.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance

    def MDSPA(self, distanceSetPointList: list[float, float, float, float] = None) -> list[float, float, float, float]:
        """

        Sets or gets the Motor Distance set point for all motors at once.

        :param distanceSetPointList:  Distance set points in mm in order [Left Top, Left Bottom, Right Top, Right Bottom]
        :return: Distance set points in mm
        """
        if distanceSetPointList is not None:
            self._MLT.encoderSetPoint(
                encoderSetPoint=int(distanceSetPointList[0] * nimbusMotorController.__distanceToEncoderCounts))
            self._MLB.encoderSetPoint(
                encoderSetPoint=int(distanceSetPointList[1] * nimbusMotorController.__distanceToEncoderCounts))
            self._MRT.encoderSetPoint(
                encoderSetPoint=int(distanceSetPointList[2] * nimbusMotorController.__distanceToEncoderCounts))
            self._MRB.encoderSetPoint(
                encoderSetPoint=int(distanceSetPointList[3] * nimbusMotorController.__distanceToEncoderCounts))
        return [self._MLT.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance,
                self._MLB.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance,
                self._MRT.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance,
                self._MRB.encoderSetPoint() * nimbusMotorController.__encoderCountsToDistance]

    def MLTMM(self, minMaxList: list[int, int] = None) -> list[int, int]:
        """
        Set or get the minimum and maximum speed of the Motor Left Top.
        :param minMaxList: motor Speeds [min, max]
        :return: returns current motor speeds [min, max]
        """
        if minMaxList is not None:
            self._MLT.minMax(minMaxList=minMaxList)
        return self._MLT.minMax()

    def MLBMM(self, minMaxList: list[int, int] = None) -> list[int, int]:
        """
        Set or get the minimum and maximum speed of the Motor Left Bottom.
        :param minMaxList: motor Speeds [min, max]
        :return: returns current motor speeds [min, max]
        """
        if minMaxList is not None:
            self._MLB.minMax(minMaxList=minMaxList)
        return self._MLB.minMax()

    def MRTMM(self, minMaxList: list[int, int] = None) -> list[int, int]:
        """
        Set or get the minimum and maximum speed of the Motor Right Top.
        :param minMaxList: motor Speeds [min, max]
        :return: returns current motor speeds [min, max]
        """
        if minMaxList is not None:
            self._MRT.minMax(minMaxList=minMaxList)
        return self._MRT.minMax()

    def MRBMM(self, minMaxList: [int, int] = None) -> [int, int]:
        """
        Set or get the minimum and maximum speed of the Motor Right Bottom.
        :param minMaxList: motor Speeds [min, max]
        :return: returns current motor speeds [min, max]
        """
        if minMaxList is not None:
            self._MRB.minMax(minMaxList=minMaxList)
        return self._MRB.minMax()

    def MMMA(self, minMaxListList: list[list[int, int], list[int, int], list[int, int], list[int, int]] = None) -> list[
        list[int, int], list[int, int], list[int, int], list[int, int]]:
        """

        Set or get the minimum and maximum speed limits of all motors.

        :param minMaxListList: minimum and maximum speeds in list
        :return: current speed limits of all.
        """
        if minMaxListList is not None:
            self._MLT.minMax(minMaxList=minMaxListList[0])
            self._MLB.minMax(minMaxList=minMaxListList[1])
            self._MRT.minMax(minMaxList=minMaxListList[2])
            self._MRB.minMax(minMaxList=minMaxListList[3])
        return [self._MLT.minMax(), self._MLB.minMax(), self._MRT.minMax(), self._MRB.minMax()]

    def MLTR(self) -> None:
        """
        Resets the Motor Left Top.

        :return: None
        """
        self._MLT.reset()

    def MLBR(self) -> None:
        """
        Resets the Motor Left Bottom.

        :return: None
        """
        self._MLB.reset()

    def MRTR(self) -> None:
        """
        Resets the Motor Right Top.

        :return: None
        """
        self._MRT.reset()

    def MRBR(self) -> None:
        """
        Resets the Motor Right Bottom.

        :return: None
        """
        self._MRB.reset()

    def MRA(self) -> None:
        """

        Resets all motors.

        :return: None
        """
        self._MLT.reset()
        self._MLB.reset()
        self._MRT.reset()
        self._MRB.reset()

    def MLTCP(self, PIDParams: [float, float, float] = None) -> [float, float, float]:
        """

        Sets or gets the PID Parameters [Kp, Ki, Kd] of Motor Left Top.

        :param PIDParams: [Kp, Ki, Kd]
        :return: current PID Parameters.
        """
        if PIDParams is not None:
            self._MLT.controllerParams(PIDParams=PIDParams)
        return self._MLT.controllerParams()

    def MLBCP(self, PIDParams: [float, float, float] = None) -> [float, float, float]:
        """

        Sets or gets the PID Parameters [Kp, Ki, Kd] of Motor Left Bottom.

        :param PIDParams: [Kp, Ki, Kd]
        :return: current PID Parameters.
        """
        if PIDParams is not None:
            self._MLB.controllerParams(PIDParams=PIDParams)
        return self._MLB.controllerParams()

    def MRTCP(self, PIDParams: [float, float, float] = None) -> [float, float, float]:
        """

        Sets or gets the PID Parameters [Kp, Ki, Kd] of Motor Right Top.

        :param PIDParams: [Kp, Ki, Kd]
        :return: current PID Parameters.
        """
        if PIDParams is not None:
            self._MRT.controllerParams(PIDParams=PIDParams)
        return self._MRT.controllerParams()

    def MRBCP(self, PIDParams: [float, float, float] = None) -> [float, float, float]:
        """

        Sets or gets the PID Parameters [Kp, Ki, Kd] of Motor Right Bottom.

        :param PIDParams: [Kp, Ki, Kd]
        :return: current PID Parameters.
        """
        if PIDParams is not None:
            self._MRB.controllerParams(PIDParams=PIDParams)
        return self._MRB.controllerParams()

    def MCPA(self, PIDParamsList: list[
        list[float, float, float], list[float, float, float], list[float, float, float], list[
            float, float, float]] = None) -> list[list[float, float, float], list[float, float, float], list[float, float, float], list[float, float, float]]:
        """

        Sets or Gets the PID Parameters of all motors at once

        :param PIDParamsList: sets the PID Parameters
        :return: current PID Parameters
        """
        if PIDParamsList is not None:
            self._MLT.controllerParams(PIDParams=PIDParamsList[0])
            self._MLB.controllerParams(PIDParams=PIDParamsList[1])
            self._MRT.controllerParams(PIDParams=PIDParamsList[2])
            self._MRB.controllerParams(PIDParams=PIDParamsList[3])
        return [self._MLT.controllerParams(),
                self._MLB.controllerParams(),
                self._MRT.controllerParams(),
                self._MRB.controllerParams()]

    def MVA(self,
            theta: float = None,
            vel: float = None,
            phi: float = None,
            timeOut: int = None):

        self._timer.init(callback=None)
        self._MSP.value(0)
        self._MLT.motorSpeed(speed=0)
        self._MLB.motorSpeed(speed=0)
        self._MRT.motorSpeed(speed=0)
        self._MRB.motorSpeed(speed=0)
        s = math.sin
        c = math.cos
        f = math.fabs
        pi = math.pi
        pi_4 = pi / 4
        if phi is not None and -pi < phi <= pi:
            turn = phi / pi
        else:
            turn = 0
        if theta is not None and 0 <= theta < 2 * pi:
            sin_comp = s(theta - pi_4)
            cos_comp = c(theta - pi_4)
            max_comp = max(f(sin_comp), f(cos_comp))
        else:
            sin_comp = 0
            cos_comp = 0
            max_comp = 1
        lt = cos_comp / max_comp - turn
        rt = sin_comp / max_comp + turn
        lb = sin_comp / max_comp - turn
        rb = cos_comp / max_comp + turn
        if phi != 0 and theta is not None:
            lt /= (1 + turn)
            lb /= (1 + turn)
            rt /= (1 + turn)
            rb /= (1 + turn)
        ltS = int(lt * vel)
        rtS = int(rt * vel)
        lbS = int(lb * vel)
        rbS = int(rb * vel)
        if timeOut is None or timeOut <= 0:
            timeOutValue = self._timeOut
        else:
            timeOutValue = max(timeOut, nimbusMotorController.__minimumTimeout)

        self._MLT.motorSpeed(speed=ltS)
        self._MLB.motorSpeed(speed=lbS)
        self._MRT.motorSpeed(speed=rtS)
        self._MRB.motorSpeed(speed=rbS)
        self._MSP.value(1)
        self._timer.init(mode=Timer.ONE_SHOT, period=timeOutValue, callback=self._selfCallBack)

    def MTA(self,
            timeOut: int = None):
        if timeOut is None or timeOut <= 0:
            self._timeOut = nimbusMotorController.__defaultTimeout
        else:
            self._timeOut = min(timeOut, nimbusMotorController.__minimumTimeout)
