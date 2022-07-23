import i2cPeripheral
from machine import Timer
from nrpd import *


class _Registers:
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
    NMAS = 0x15
    NMLTTS = 0x19
    NMLBTS = 0x1D
    NMRTTS = 0x21
    NMRBTS = 0x25
    NMLTTR = 0x29
    NMLBTR = 0x2D
    NMRTTR = 0x31
    NMRBTR = 0x35
    NRTSP = 0x39
    NRTAD = 0x3A
    NRTTS = 0x3C
    NBTN = 0x3D
    DC = 61


class nimbusI2CPeripheral:

    def __init__(self, callBackFunction, initialBatteryCutoff, frequency=50, i2cAddress=0x55):
        self.regs = _Registers()
        self.singeByteRegisters = [self.regs.NLED, self.regs.NMSC, # RW 1 Byte
                                   self.regs.NBTN, self.regs.NRTSP, self.regs.NRTTS]
        self.doubleByteRegisters = [self.regs.NBLV, self.regs.NBCV, self.regs.NPBV, self.regs.NMLTC,
                                    self.regs.NMLBC, self.regs.NMRTC, self.regs.NMRBC, self.regs.NRTAD]  # RW 2 Bytes
        self.tripleByteRegisters = [self.regs.NNPXL]  # RW 3 Bytes
        self.quadByteRegisters = [self.regs.NMAS,  # RW 4 Bytes
                                  self.regs.NMLTTS, self.regs.NMLBTS, self.regs.NMRTTS, self.regs.NMRBTS,
                                  self.regs.NMLTTR, self.regs.NMLBTR, self.regs.NMRTTR, self.regs.NMRBTR]

        # check sign before assigning to motor controller for NMXXTS data
        # Store Packed Bytes correctly in NMXXTR locations.
        self.encoderRegister = [self.regs.NMLTTS, self.regs.NMLBTS, self.regs.NMRTTS, self.regs.NMRBTS,
                                self.regs.NMLTTR, self.regs.NMLBTR, self.regs.NMRTTR, self.regs.NMRBTR] # Store bytes.
        self.dataFieldTx = bytearray([0] * self.regs.DC)
        self.dataFieldRx = bytearray([0] * self.regs.DC)
        self.dataFieldTx[self.regs.NBCV - self.regs.OFFSET] = initialBatteryCutoff[0]
        self.dataFieldTx[self.regs.NBCV - self.regs.OFFSET + 1] = initialBatteryCutoff[1]
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
        elif register in self.quadByteRegisters:
            count = 4
        if count is 1:
            self.dataFieldRx[register - self.regs.OFFSET] = self.p_i2c.get()
        else:
            size = self.p_i2c.get()  # should be equal to count.
            for i in range(0, count):
                self.dataFieldRx[register - self.regs.OFFSET + i] = self.p_i2c.get()
        for i in range(0, count):
            self.dataFieldTx[register - self.regs.OFFSET + i] = self.dataFieldRx[register - self.regs.OFFSET + i]
