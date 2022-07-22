# pico toggle onboard LED on-off

import machine
# import time
# from machine import mem32, mem8, Timer
import time

import math

from lib import i2cPeripheral, adcWTMux, ws2812b
from machine import Timer
from nrpd import *

led_onboard = machine.Pin(19, machine.Pin.OUT)
motorSwitchControl = machine.Pin(MOTOR_SWITCH_PIN, machine.Pin.OUT)
button = machine.Pin(BUTTON_PIN, machine.Pin.IN)
adc = adcWTMux.adcWTMux()
batteryCutoffVoltage = 3.8


class Registers:
    OFFSET = 1
    NR = 0x01
    NBLV = 0x02
    NPBV = 0x04
    NBCV = 0x06
    NMLTC = 0x08
    NMLBC = 0x0A
    NMRTC = 0x0C
    NMRBC = 0x0E
    NNPXL = 0x10
    NLED = 0x13
    NMSC = 0x14
    NMLTS = 0x15
    NMLBS = 0x16
    NMRTS = 0x17
    NMRBS = 0x18
    NMLTT = 0x19
    NMLBT = 0x1B
    NMRTT = 0x1D
    NMRBT = 0x1F
    NMTSP = 0x21
    NMTAR = 0x22
    NMTTS = 0x24
    NBTN = 0x25
    DC = 37


def dataToAnalogConverter(data, scale):
    return (data[0] << 8 | data[1]) / scale


def analogToDataConverter(analogValue, scale):
    data = [0, 0]
    analogValue = math.ceil(analogValue * scale)
    data[0] = (analogValue & 0xFF00) >> 8
    data[1] = analogValue & 0x00FF
    return data


class nimbusI2CPeripheral:

    def __init__(self, callBackFunction):
        self.regs = Registers()
        self.singeByteRegisters = [self.regs.NLED, self.regs.NMSC, self.regs.NBTN]
        self.doubleByteRegisters = [self.regs.NBLV, self.regs.NBCV, self.regs.NPBV, self.regs.NMLTC, self.regs.NMLBC,
                                    self.regs.NMRTC, self.regs.NMRBC]
        self.tripleByteRegisters = [self.regs.NNPXL]
        self.dataFieldTx = bytearray([0] * self.regs.DC)
        self.dataFieldRx = bytearray([0] * self.regs.DC)
        data = analogToDataConverter(batteryCutoffVoltage, 100)
        self.dataFieldTx[self.regs.NBCV - self.regs.OFFSET] = data[0]
        self.dataFieldTx[self.regs.NBCV - self.regs.OFFSET + 1] = data[1]
        self.p_i2c = i2cPeripheral.i2cPeripheral(i2cID=0, sda=I2C_SDA, scl=I2C_SCL, peripheralAddress=0x55)
        self.tim = Timer()
        self.tim.init(freq=50, mode=Timer.PERIODIC, callback=callBackFunction)

    def write(self, register):
        count = 0
        if register in self.singeByteRegisters:
            count = 1
        elif register in self.doubleByteRegisters:
            count = 2
        elif register in self.tripleByteRegisters:
            count = 3
        for i in range(0, count):
            self.p_i2c.put(self.dataFieldTx[register - self.regs.OFFSET + i])

    def read(self, register):
        count = 0
        if register in self.singeByteRegisters:
            count = 1
        elif register in self.doubleByteRegisters:
            count = 2
        elif register in self.tripleByteRegisters:
            count = 3
        if count is 1:
            self.dataFieldRx[register - self.regs.OFFSET] = self.p_i2c.get()
        else:
            size = self.p_i2c.get()  # should be equal to count.
            for i in range(0, count):
                self.dataFieldRx[register - self.regs.OFFSET + i] = self.p_i2c.get()
        for i in range(0, count):
            self.dataFieldTx[register - self.regs.OFFSET + i] = self.dataFieldRx[register - self.regs.OFFSET + i]


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
            reg = i2cPeripheralHandler.p_i2c.get()  # Read the incoming byte from controller so controller doesn't
            # stay waiting.
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
                nnpxlPos = i2cPeripheralHandler.regs.NNPXL - i2cPeripheralHandler.regs.OFFSET
                ws2812b.pixels_set(0, (
                    i2cPeripheralHandler.dataFieldRx[nnpxlPos], i2cPeripheralHandler.dataFieldRx[nnpxlPos + 1],
                    i2cPeripheralHandler.dataFieldRx[nnpxlPos + 2]))
                ws2812b.pixels_show()
            elif reg is i2cPeripheralHandler.regs.NBCV:
                batteryCutoffVoltage = dataToAnalogConverter(i2cPeripheralHandler.dataFieldRx[
                                                             i2cPeripheralHandler.regs.NBCV - i2cPeripheralHandler.regs.OFFSET:i2cPeripheralHandler.regs.NBCV - i2cPeripheralHandler.regs.OFFSET + 2],
                                                             100)


i2cPeripheralHandler = nimbusI2CPeripheral(callBackFunction=callBackFunc)


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
