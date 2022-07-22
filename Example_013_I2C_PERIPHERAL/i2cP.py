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
    global i2cHandler, batteryCutoffVoltage
    rxS = i2cHandler.p_i2c.rxFifoSize()
    wI = i2cHandler.p_i2c.anyRead()
    rI = i2cHandler.p_i2c.any()
    if rxS is 0 and wI and not rI:
        i2cHandler.p_i2c.put(0x00)  # For general call release
    if rI:
        readAv = True
        reg = i2cHandler.p_i2c.get()
        if reg == i2cHandler.regs.NR:
            reg = i2cHandler.p_i2c.get()  # Read the incoming byte from controller so controller doesn't stay waiting.
            machine.reset()
        if wI:
            i2cHandler.write(reg)
        else:
            i2cHandler.read(reg)
            if reg is i2cHandler.regs.NLED:
                led_onboard.value(i2cHandler.dataFieldRx[i2cHandler.regs.NLED - i2cHandler.regs.OFFSET])
            elif reg is i2cHandler.regs.NMSC:
                motorSwitchControl.value(i2cHandler.dataFieldRx[i2cHandler.regs.NMSC - i2cHandler.regs.OFFSET])
            elif reg is i2cHandler.regs.NNPXL:
                nnpxlPos = i2cHandler.regs.NNPXL - i2cHandler.regs.OFFSET
                ws2812b.pixels_set(0, (i2cHandler.dataFieldRx[nnpxlPos], i2cHandler.dataFieldRx[nnpxlPos + 1],
                                       i2cHandler.dataFieldRx[nnpxlPos + 2]))
                ws2812b.pixels_show()
            elif reg is i2cHandler.regs.NBCV:
                batteryCutoffVoltage = dataToAnalogConverter(i2cHandler.dataFieldRx[
                                                             i2cHandler.regs.NBCV - i2cHandler.regs.OFFSET:i2cHandler.regs.NBCV - i2cHandler.regs.OFFSET + 2],
                                                             100)


i2cHandler = nimbusI2CPeripheral(callBackFunction=callBackFunc)


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
        blPos = i2cHandler.regs.NBLV - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[blPos] = dataB[0]
        i2cHandler.dataFieldTx[blPos + 1] = dataB[1]

        dataP = analogToDataConverter(boostLevel, 100)
        boostLPos = i2cHandler.regs.NPBV - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[boostLPos] = dataP[0]
        i2cHandler.dataFieldTx[boostLPos + 1] = dataP[1]

        datamlt = analogToDataConverter(mltC, 1)
        mltPos = i2cHandler.regs.NMLTC - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[mltPos] = datamlt[0]
        i2cHandler.dataFieldTx[mltPos + 1] = datamlt[1]

        datamlb = analogToDataConverter(mlbC, 1)
        mlbPos = i2cHandler.regs.NMLBC - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[mlbPos] = datamlb[0]
        i2cHandler.dataFieldTx[mlbPos + 1] = datamlb[1]

        datamrt = analogToDataConverter(mrtC, 1)
        mrtPos = i2cHandler.regs.NMRTC - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[mrtPos] = datamrt[0]
        i2cHandler.dataFieldTx[mrtPos + 1] = datamrt[1]

        datamrb = analogToDataConverter(mrbC, 1)
        mrbPos = i2cHandler.regs.NMRBC - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[mrbPos] = datamrb[0]
        i2cHandler.dataFieldTx[mrbPos + 1] = datamrb[1]

        btnPos = i2cHandler.regs.NBTN - i2cHandler.regs.OFFSET
        i2cHandler.dataFieldTx[btnPos] = button.value()

        # print("Battery: {B:.2f}\tPowerBooster: {P:.2f} Cutoff: {C:.2f}".format(B=battLevel, P=boostLevel,
        #                                                                        C=batteryCutoffVoltage), dataB, dataP)
        time.sleep(0.5)


looper()

"""
p_i2c = i2cPeripheral.i2cPeripheral(i2cID=0, sda=20, scl=21, peripheralAddress=0x55)
counter = 0
readValue = 0
readAv = False
writeAv = False

def i2cRec(t):
    global counter, readAv, readValue
    rxS = p_i2c.rxFifoSize()
    if rxS is not 0:
        print("rxS: {}".format(rxS))
    if p_i2c.any():
        readAv = True
        for i in range(0, rxS):
            readValue = p_i2c.get()
            print("Read: {}".format(readValue))
    if p_i2c.anyRead():
        counter = counter + 1
        p_i2c.put(counter)
        p_i2c.put(counter + 1)
        print("Read: {}".format(readAv), "ReadValue: {}".format(readValue), "Writen: {}".format(counter))


tim = Timer()
tim.init(freq=50, mode=Timer.PERIODIC, callback=i2cRec)

try:
    while True:
        print(readValue, readAv)
        if readAv:
            if readValue == 12:
                led_onboard.value(1)
            elif readValue == 15:
                led_onboard.value(0)
            readAv = False
        motorSwitchControl.toggle()
        time.sleep(1)

except KeyboardInterrupt:
    pass
"""
