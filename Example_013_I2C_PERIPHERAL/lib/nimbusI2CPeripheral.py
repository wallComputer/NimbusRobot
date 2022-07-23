import i2cPeripheral
from machine import Timer
from nrpd import *
import math


def list2ToContinuousConverter(data, scale):
    return (data[0] << 8 | data[1]) / scale


def continuousToList2Converter(analogValue, scale):
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


def list4ToContinuousConverter(data):
    return negativeTester(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3], 4)


def continuousTolist4Converter(encoderValue):
    data = [0, 0, 0, 0]
    data[0] = (encoderValue & 0xFF000000) >> 24
    data[1] = (encoderValue & 0x00FF0000) >> 16
    data[2] = (encoderValue & 0x0000FF00) >> 8
    data[3] = (encoderValue & 0x000000FF)
    return data


class _Registers:
    NR = 0x00
    NBLV = 0x01
    NPBV = 0x03
    NBCV = 0x05
    NMLTC = 0x07
    NMLBC = 0x09
    NMRTC = 0x0B
    NMRBC = 0x0D
    NNPXL = 0x0F
    NLED = 0x12
    NMSC = 0x13
    NMAS = 0x14
    NMLTTS = 0x18
    NMLBTS = 0x1C
    NMRTTS = 0x20
    NMRBTS = 0x24
    NMLTTR = 0x28
    NMLBTR = 0x2C
    NMRTTR = 0x30
    NMRBTR = 0x34
    NRTSP = 0x38
    NRTAD = 0x39
    NRTTMS = 0x3B
    NBTN = 0x3D
    DC = 62


class nimbusI2CPeripheral:

    def __init__(self, callBackFunction, initialBatteryCutoff, frequency=50, i2cAddress=0x55):
        self.regs = _Registers()
        self.singeByteRegisters = [self.regs.NLED, self.regs.NMSC,  # RW 1 Byte
                                   self.regs.NBTN, self.regs.NRTSP]
        self.doubleByteRegisters = [self.regs.NBLV, self.regs.NBCV,
                                    self.regs.NPBV, self.regs.NMLTC,
                                    self.regs.NMLBC, self.regs.NMRTC,
                                    self.regs.NMRBC, self.regs.NRTAD, self.regs.NRTTMS]  # RW 2 Bytes
        self.tripleByteRegisters = [self.regs.NNPXL]  # RW 3 Bytes
        self.quadByteRegisters = [self.regs.NMAS, self.regs.NMLTTS,
                                  self.regs.NMLBTS, self.regs.NMRTTS,
                                  self.regs.NMRBTS, self.regs.NMLTTR,
                                  self.regs.NMLBTR, self.regs.NMRTTR, self.regs.NMRBTR]  # RW 4 Bytes

        # check sign before assigning to motor controller for NMXXTS data
        # Store Packed Bytes correctly in NMXXTR locations.
        self.encoderRegister = [self.regs.NMLTTS, self.regs.NMLBTS, self.regs.NMRTTS, self.regs.NMRBTS,
                                self.regs.NMLTTR, self.regs.NMLBTR, self.regs.NMRTTR, self.regs.NMRBTR]  # Store bytes.
        self.dataFieldTx = bytearray([0] * self.regs.DC)
        self.dataFieldRx = bytearray([0] * self.regs.DC)
        self.dataFieldTx[self.regs.NBCV:self.regs.NBCV + 2] = bytearray(continuousToList2Converter(initialBatteryCutoff, 100))
        self.p_i2c = i2cPeripheral.i2cPeripheral(i2cID=0, sda=I2C_SDA, scl=I2C_SCL, peripheralAddress=i2cAddress)
        self.tim = Timer()
        self.tim.init(freq=frequency, mode=Timer.PERIODIC, callback=callBackFunction)

    def write(self, register):
        count = 0
        if register in self.singeByteRegisters:
            count = 1
        elif register in self.doubleByteRegisters:
            count = 2
        elif register in self.tripleByteRegisters:
            count = 3
        elif register in self.quadByteRegisters:
            count = 4
        print("WF: {0}, C: {1}".format(hex(register), count))
        for i in range(0, count):
            self.p_i2c.put(self.dataFieldTx[register + i])

    def read(self, register):
        count = 0
        if register in self.singeByteRegisters:
            count = 1
        elif register in self.doubleByteRegisters:
            count = 2
        elif register in self.tripleByteRegisters:
            count = 3
        elif register in self.quadByteRegisters:
            count = 4

        print("RF: {0}, C: {1}".format(hex(register), count))
        if count is 1:
            self.dataFieldRx[register] = self.p_i2c.get()
        else:
            size = self.p_i2c.get()  # should be equal to count.
            for i in range(0, count):
                self.dataFieldRx[register + i] = self.p_i2c.get()
        for i in range(0, count):
            self.dataFieldTx[register + i] = self.dataFieldRx[register + i]
