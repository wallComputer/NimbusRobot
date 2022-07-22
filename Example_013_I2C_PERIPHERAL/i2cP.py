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


def dataToAnalogConverter(data, scale):
    return (data[0] << 8 | data[1]) / scale


def analogToDataConverter(analogValue, scale):
    data = [0, 0]
    analogValue = math.ceil(analogValue * scale)
    data[0] = (analogValue & 0xFF00) >> 8
    data[1] = analogValue & 0x00FF
    return data


def callBackFunc(tim):
    global i2cPeripheralHandler, batteryCutoffVoltage
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
                batteryCutoffVoltage = dataToAnalogConverter(i2cPeripheralHandler.dataFieldRx[bcvPos:bcvPos + 2], 100)


i2cPeripheralHandler = nimbusI2CPeripheral(callBackFunction=callBackFunc,
                                           initialBatteryCutoff=analogToDataConverter(batteryCutoffVoltage, 100),
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

        dataB = analogToDataConverter(battLevel, 100)
        blPos = i2cPeripheralHandler.regs.NBLV - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[blPos] = dataB[0]
        i2cPeripheralHandler.dataFieldTx[blPos + 1] = dataB[1]

        dataP = analogToDataConverter(boostLevel, 100)
        boostLPos = i2cPeripheralHandler.regs.NPBV - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[boostLPos] = dataP[0]
        i2cPeripheralHandler.dataFieldTx[boostLPos + 1] = dataP[1]

        datamlt = analogToDataConverter(mltC, 1)
        mltPos = i2cPeripheralHandler.regs.NMLTC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mltPos] = datamlt[0]
        i2cPeripheralHandler.dataFieldTx[mltPos + 1] = datamlt[1]

        datamlb = analogToDataConverter(mlbC, 1)
        mlbPos = i2cPeripheralHandler.regs.NMLBC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mlbPos] = datamlb[0]
        i2cPeripheralHandler.dataFieldTx[mlbPos + 1] = datamlb[1]

        datamrt = analogToDataConverter(mrtC, 1)
        mrtPos = i2cPeripheralHandler.regs.NMRTC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mrtPos] = datamrt[0]
        i2cPeripheralHandler.dataFieldTx[mrtPos + 1] = datamrt[1]

        datamrb = analogToDataConverter(mrbC, 1)
        mrbPos = i2cPeripheralHandler.regs.NMRBC - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[mrbPos] = datamrb[0]
        i2cPeripheralHandler.dataFieldTx[mrbPos + 1] = datamrb[1]

        btnPos = i2cPeripheralHandler.regs.NBTN - i2cPeripheralHandler.regs.OFFSET
        i2cPeripheralHandler.dataFieldTx[btnPos] = button.value()

        # print("Battery: {B:.2f}\tPowerBooster: {P:.2f} Cutoff: {C:.2f}".format(B=battLevel, P=boostLevel,
        #                                                                        C=batteryCutoffVoltage), dataB, dataP)
        time.sleep(0.5)


looper()
