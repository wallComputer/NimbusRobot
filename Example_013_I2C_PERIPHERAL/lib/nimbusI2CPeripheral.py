import i2cPeripheral
from machine import Timer
from nrpd import *
import math
import struct


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
    NLED = 0x01
    NMSP = 0x02
    NBTN = 0x03
    NNPXL = 0x04
    NBLV = 0x07
    NPBV = 0x0B
    NBCV = 0x0F
    NMLTC = 0x13
    NMLBC = 0x17
    NMRTC = 0x1B
    NMRBC = 0x1F
    NMLTS = 0x23
    NMLBS = 0x27
    NMRTS = 0x2B
    NMRBS = 0x2F
    NMTVPT = 0x33
    NMT = 0x3F
    NMTB = 0x43
    DC = 68


class nimbusI2CPeripheral:

    # noinspection PyArgumentList
    def __init__(self, callBackFunction, initialBatteryCutoff, initialRobotTwistTime=1500,
                 frequency=50, i2cAddress=0x55):
        self.regs = _Registers()
        self.singeByteRegisters = [self.regs.NLED, self.regs.NMSP, self.regs.NBTN, self.regs.NMTB]  # RW 1 Byte
        self.tripleByteRegisters = [self.regs.NNPXL]  # RW 3 Bytes
        self.floatRegisters = [self.regs.NBLV, self.regs.NBCV, self.regs.NPBV,
                               self.regs.NMLTC, self.regs.NMLBC, self.regs.NMRTC, self.regs.NMRBC,
                               self.regs.NMLTS, self.regs.NMLBS, self.regs.NMRTS, self.regs.NMRBS,
                               self.regs.NMT]  # RW 4 bytes
        self.sedecimByteRegisters = [self.regs.NMTVPT]

        self.dataFieldTx = bytearray([0] * self.regs.DC)
        self.dataFieldRx = bytearray([0] * self.regs.DC)
        self.dataFieldTx[self.regs.NBCV: self.regs.NBCV+4] = struct.pack('f', initialBatteryCutoff)
        self.p_i2c = i2cPeripheral.i2cPeripheral(i2cID=0, sda=I2C_SDA, scl=I2C_SCL, peripheralAddress=i2cAddress)
        self.tim = Timer()
        self.tim.init(period=int(1000/frequency), mode=Timer.PERIODIC, callback=callBackFunction)

    def writeToController(self, register):
        count = 0
        if register in self.singeByteRegisters:
            count = 1
        elif register in self.tripleByteRegisters:
            count = 3
        elif register in self.floatRegisters:
            count = 4
        elif register in self.sedecimByteRegisters:
            count = 12
        print("WT: {0}, C: {1}".format(hex(register), count))
        for i in range(0, count):
            self.p_i2c.put(self.dataFieldTx[register + i])

    def readFromController(self, register):
        count = 0
        if register in self.singeByteRegisters:
            count = 1
        elif register in self.tripleByteRegisters:
            count = 3
        elif register in self.floatRegisters:
            count = 4
        elif register in self.sedecimByteRegisters:
            count = 12
        print("RF: {0}, C: {1}".format(hex(register), count))
        if count is 1:
            self.dataFieldRx[register] = self.p_i2c.get()
        else:
            size = self.p_i2c.get()  # should be equal to count.
            for i in range(0, count):
                self.dataFieldRx[register + i] = self.p_i2c.get()
        for i in range(0, count):
            self.dataFieldTx[register + i] = self.dataFieldRx[register + i]

    def fillTxFloat(self, register, value):
        self.dataFieldTx[register:register + 4] = struct.pack('f', value)
