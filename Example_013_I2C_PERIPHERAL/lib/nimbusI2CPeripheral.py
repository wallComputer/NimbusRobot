
import i2cPeripheral
from machine import Timer
from nrpd import *

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


class nimbusI2CPeripheral:

    def __init__(self, callBackFunction, initialBatteryCutoff):
        self.regs = Registers()
        self.singeByteRegisters = [self.regs.NLED, self.regs.NMSC, self.regs.NBTN]
        self.doubleByteRegisters = [self.regs.NBLV, self.regs.NBCV, self.regs.NPBV,
                                    self.regs.NMLTC, self.regs.NMLBC,
                                    self.regs.NMRTC, self.regs.NMRBC]
        self.tripleByteRegisters = [self.regs.NNPXL]
        self.dataFieldTx = bytearray([0] * self.regs.DC)
        self.dataFieldRx = bytearray([0] * self.regs.DC)
        self.dataFieldTx[self.regs.NBCV - self.regs.OFFSET] = initialBatteryCutoff[0]
        self.dataFieldTx[self.regs.NBCV - self.regs.OFFSET + 1] = initialBatteryCutoff[1]
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
