# pico toggle onboard LED on-off

import machine
import time

from lib import adcWTMux, ws2812b, motors
from nrpd import *

from lib.nimbusI2CPeripheral import nimbusI2CPeripheral, negativeTester, \
    list2ToContinuousConverter, continuousToList2Converter

led_onboard = machine.Pin(19, machine.Pin.OUT)
button = machine.Pin(BUTTON_PIN, machine.Pin.IN)
adc = adcWTMux.adcWTMux()
batteryCutoffVoltage = 3.8
motorController = motors.motorControllers()
robotTwistSpeed = 0
robotTwistAngle = 0
robotTwistTime = 1500

# TODO: Go to Motor Encoders, add callback to check immediate change.
# TODO Add those callbacks here to update i2cPeripheralHandler.dataFieldRx immediately.
def callBackFunc(tim):
    global i2cPeripheralHandler, batteryCutoffVoltage, motorController, robotTwistSpeed, robotTwistAngle, robotTwistTime
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
                    i2cPeripheralHandler.dataFieldRx[reg])
            elif reg is i2cPeripheralHandler.regs.NMSC:
                motorController.MSP = i2cPeripheralHandler.dataFieldRx[reg]
            elif reg is i2cPeripheralHandler.regs.NNPXL:
                ws2812b.pixels_set(0, (i2cPeripheralHandler.dataFieldRx[reg], i2cPeripheralHandler.dataFieldRx[reg + 1],
                    i2cPeripheralHandler.dataFieldRx[reg + 2]))
                ws2812b.pixels_show()
            elif reg is i2cPeripheralHandler.regs.NBCV:
                batteryCutoffVoltage = list2ToContinuousConverter(i2cPeripheralHandler.dataFieldRx[reg:reg + 2], 100)
            elif reg is i2cPeripheralHandler.regs.NMAS:
                motorController.MLT = negativeTester(i2cPeripheralHandler.dataFieldRx[reg], 1)
                motorController.MLB = negativeTester(i2cPeripheralHandler.dataFieldRx[reg + 1], 1)
                motorController.MRT = negativeTester(i2cPeripheralHandler.dataFieldRx[reg + 2], 1)
                motorController.MRB = negativeTester(i2cPeripheralHandler.dataFieldRx[reg + 3], 1)
            elif reg is i2cPeripheralHandler.regs.NRTSP:
                robotTwistSpeed = i2cPeripheralHandler.dataFieldRx[reg]
            elif reg is i2cPeripheralHandler.regs.NRTAD:
                robotTwistAngle = negativeTester(int(
                    list2ToContinuousConverter(i2cPeripheralHandler.dataFieldRx[reg:reg + 2], 1)), 2)
            elif reg is i2cPeripheralHandler.regs.NRTTMS:
                robotTwistTime = int(list2ToContinuousConverter(i2cPeripheralHandler.dataFieldRx[reg:reg + 2],1))
                print(robotTwistTime)
            elif reg in i2cPeripheralHandler.encoderRegister:
                if reg is i2cPeripheralHandler.regs.NMLTS:
                    pass



i2cPeripheralHandler = nimbusI2CPeripheral(callBackFunction=callBackFunc,
                                           initialBatteryCutoff=batteryCutoffVoltage,
                                           frequency=100,
                                           initialRobotTwistTime=robotTwistTime,
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

        dataB = continuousToList2Converter(battLevel, 100)
        blPos = i2cPeripheralHandler.regs.NBLV
        i2cPeripheralHandler.dataFieldTx[blPos] = dataB[0]
        i2cPeripheralHandler.dataFieldTx[blPos + 1] = dataB[1]

        dataP = continuousToList2Converter(boostLevel, 100)
        boostLPos = i2cPeripheralHandler.regs.NPBV
        i2cPeripheralHandler.dataFieldTx[boostLPos] = dataP[0]
        i2cPeripheralHandler.dataFieldTx[boostLPos + 1] = dataP[1]

        datamlt = continuousToList2Converter(mltC, 1)
        mltPos = i2cPeripheralHandler.regs.NMLTC
        i2cPeripheralHandler.dataFieldTx[mltPos] = datamlt[0]
        i2cPeripheralHandler.dataFieldTx[mltPos + 1] = datamlt[1]

        datamlb = continuousToList2Converter(mlbC, 1)
        mlbPos = i2cPeripheralHandler.regs.NMLBC
        i2cPeripheralHandler.dataFieldTx[mlbPos] = datamlb[0]
        i2cPeripheralHandler.dataFieldTx[mlbPos + 1] = datamlb[1]

        datamrt = continuousToList2Converter(mrtC, 1)
        mrtPos = i2cPeripheralHandler.regs.NMRTC
        i2cPeripheralHandler.dataFieldTx[mrtPos] = datamrt[0]
        i2cPeripheralHandler.dataFieldTx[mrtPos + 1] = datamrt[1]

        datamrb = continuousToList2Converter(mrbC, 1)
        mrbPos = i2cPeripheralHandler.regs.NMRBC
        i2cPeripheralHandler.dataFieldTx[mrbPos] = datamrb[0]
        i2cPeripheralHandler.dataFieldTx[mrbPos + 1] = datamrb[1]

        btnPos = i2cPeripheralHandler.regs.NBTN
        i2cPeripheralHandler.dataFieldTx[btnPos] = button.value()

        time.sleep(0.5)


looper()
