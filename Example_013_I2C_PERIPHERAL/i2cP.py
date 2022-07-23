# pico toggle onboard LED on-off

import machine
import time
import math

from lib import adcWTMux, ws2812b
from nrpd import *

from lib.nimbusI2CPeripheral import nimbusI2CPeripheral

led_onboard = machine.Pin(19, machine.Pin.OUT)
motorSwitchControl = machine.Pin(MOTOR_SWITCH_PIN, machine.Pin.OUT)
button = machine.Pin(BUTTON_PIN, machine.Pin.IN)
adc = adcWTMux.adcWTMux()
batteryCutoffVoltage = 3.8
motorSpeeds = [0, 0, 0, 0]
robotTwistSpeed = 0
robotTwistAngle = 0


# TODO: Move these five functions to nimbusI2CPeripheral class.
# TODO: CHange names for listX
def list2ToAnalogConverter(data, scale):
    return (data[0] << 8 | data[1]) / scale


def analogToList2Converter(analogValue, scale):
    data = [0, 0]
    analogValue = math.ceil(analogValue * scale)
    data[0] = (analogValue & 0xFF00) >> 8
    data[1] = analogValue & 0x00FF
    return data


def negativeTester(data, size):
    halfSize = (1 << (size * 8)) // 2
    if data & halfSize is not 0:
        data = -1 * (halfSize * 2 - data)
    return data


def list4ToEncoderValueConverter(data):
    return negativeTester(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3], 4)


def encoderValueTolist4Converter(encoderValue):
    data = [0, 0, 0, 0]
    data[0] = (encoderValue & 0xFF000000) >> 24
    data[1] = (encoderValue & 0x00FF0000) >> 16
    data[2] = (encoderValue & 0x0000FF00) >> 8
    data[3] = (encoderValue & 0x000000FF)
    return data


def callBackFunc(tim):
    global i2cPeripheralHandler, batteryCutoffVoltage, motorSpeeds, robotTwistSpeed, robotTwistAngle
    rxS = i2cPeripheralHandler.p_i2c.rxFifoSize()
    wI = i2cPeripheralHandler.p_i2c.anyRead()
    rI = i2cPeripheralHandler.p_i2c.any()
    if rxS is 0 and wI and not rI:
        i2cPeripheralHandler.p_i2c.put(0x00)  # For general call release
    if rI:
        readAv = True
        reg = i2cPeripheralHandler.p_i2c.get()
        if reg == i2cPeripheralHandler.regs.NR:
            reg = i2cPeripheralHandler.p_i2c.get()  # Read the incoming byte so controller doesn't stay waiting.
            machine.reset()
        if wI:
            i2cPeripheralHandler.write(reg)
        else:
            i2cPeripheralHandler.read(reg)
            if reg is i2cPeripheralHandler.regs.NLED:
                led_onboard.value(
                    i2cPeripheralHandler.dataFieldRx[i2cPeripheralHandler.regs.NLED - i2cPeripheralHandler.regs.OFFSET])
            elif reg is i2cPeripheralHandler.regs.NMSC:
                motorSwitchControl.value(
                    i2cPeripheralHandler.dataFieldRx[i2cPeripheralHandler.regs.NMSC - i2cPeripheralHandler.regs.OFFSET])
            elif reg is i2cPeripheralHandler.regs.NNPXL:
                npxPos = i2cPeripheralHandler.regs.NNPXL - i2cPeripheralHandler.regs.OFFSET
                ws2812b.pixels_set(0, (
                    i2cPeripheralHandler.dataFieldRx[npxPos], i2cPeripheralHandler.dataFieldRx[npxPos + 1],
                    i2cPeripheralHandler.dataFieldRx[npxPos + 2]))
                ws2812b.pixels_show()
            elif reg is i2cPeripheralHandler.regs.NBCV:
                bcvPos = i2cPeripheralHandler.regs.NBCV - i2cPeripheralHandler.regs.OFFSET
                batteryCutoffVoltage = list2ToAnalogConverter(i2cPeripheralHandler.dataFieldRx[bcvPos:bcvPos + 2], 100)
            elif reg is i2cPeripheralHandler.regs.NMAS:
                mltPos = reg - i2cPeripheralHandler.regs.OFFSET
                motorSpeeds[0] = negativeTester(i2cPeripheralHandler.dataFieldRx[mltPos], 1)
                motorSpeeds[1] = negativeTester(i2cPeripheralHandler.dataFieldRx[mltPos + 1], 1)
                motorSpeeds[2] = negativeTester(i2cPeripheralHandler.dataFieldRx[mltPos + 2], 1)
                motorSpeeds[3] = negativeTester(i2cPeripheralHandler.dataFieldRx[mltPos + 3], 1)
            elif reg is i2cPeripheralHandler.regs.NRTSP:
                tspPos = reg - i2cPeripheralHandler.regs.OFFSET
                robotTwistSpeed = i2cPeripheralHandler.dataFieldRx[tspPos]
            elif reg is i2cPeripheralHandler.regs.NRTAD:
                tarPos = reg - i2cPeripheralHandler.regs.OFFSET
                robotTwistAngle = negativeTester(int(list2ToAnalogConverter(i2cPeripheralHandler.dataFieldRx[tarPos:tarPos+2], 1)), 2)
                print(robotTwistAngle)


i2cPeripheralHandler = nimbusI2CPeripheral(callBackFunction=callBackFunc,
                                           initialBatteryCutoff=analogToList2Converter(batteryCutoffVoltage, 100),
                                           i2cAddress=0x45)


def looper():
    while True:
        # motorSwitchControl.toggle()
        battLevel = adc.readBatteryLevel_V()
        boostLevel = adc.readPowerBooster_V()
        mltC = adc.readMotorLeftTopCurrent_mA()
        mlbC = adc.readMotorLeftBottomCurrent_mA()
        mrtC = adc.readMotorRightTopCurrent_mA()
        mrbC = adc.readMotorRightBottomCurrent_mA()

        dataB = analogToList2Converter(battLevel, 100)
        blPos = i2cPeripheralHandler.regs.NBLV - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[blPos] = dataB[0]
        i2cPeripheralHandler.dataFieldTx[blPos + 1] = dataB[1]

        dataP = analogToList2Converter(boostLevel, 100)
        boostLPos = i2cPeripheralHandler.regs.NPBV - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[boostLPos] = dataP[0]
        i2cPeripheralHandler.dataFieldTx[boostLPos + 1] = dataP[1]

        datamlt = analogToList2Converter(mltC, 1)
        mltPos = i2cPeripheralHandler.regs.NMLTC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mltPos] = datamlt[0]
        i2cPeripheralHandler.dataFieldTx[mltPos + 1] = datamlt[1]

        datamlb = analogToList2Converter(mlbC, 1)
        mlbPos = i2cPeripheralHandler.regs.NMLBC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mlbPos] = datamlb[0]
        i2cPeripheralHandler.dataFieldTx[mlbPos + 1] = datamlb[1]

        datamrt = analogToList2Converter(mrtC, 1)
        mrtPos = i2cPeripheralHandler.regs.NMRTC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mrtPos] = datamrt[0]
        i2cPeripheralHandler.dataFieldTx[mrtPos + 1] = datamrt[1]

        datamrb = analogToList2Converter(mrbC, 1)
        mrbPos = i2cPeripheralHandler.regs.NMRBC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mrbPos] = datamrb[0]
        i2cPeripheralHandler.dataFieldTx[mrbPos + 1] = datamrb[1]

        btnPos = i2cPeripheralHandler.regs.NBTN - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[btnPos] = button.value()

        time.sleep(0.5)


looper()
