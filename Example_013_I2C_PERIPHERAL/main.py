import struct
import ujson
import machine
from machine import UART
import time

from lib import adcWTMux, motorHandler, ws2812b, nimbusI2CPeripheral
from nrpd import *

led_onboard = machine.Pin(19, machine.Pin.OUT)
button = machine.Pin(BUTTON_PIN, machine.Pin.IN)
adc = adcWTMux.adcWTMux()
batteryCutoffVoltage = 3.8
robotTwistSpeed = 0
robotTwistAngle = 0
robotTwistTime = 2500

uart = UART(0, baudrate=115200, tx=machine.Pin(UART_TX), rx=machine.Pin(UART_RX))
ws2812b.pixels_show()

LTParams = {
    'motorID': 0,
    'MPINS': [MLTIN1, MLTIN2],
    'motorFreq': 75,
    'sm_no': -1,
}
LBParams = {
    'motorID': 1,
    'MPINS': [MLBIN1, MLBIN2],
    'motorFreq': 75,
    'sm_no': -1,
}
RTParams = {
    'motorID': 2,
    'MPINS': [MRTIN1, MRTIN2],
    'motorFreq': 75,
}
RBParams = {
    'motorID': 3,
    'MPINS': [MRBIN1, MRBIN2],
    'motorFreq': 75,
}
mc = motorHandler.nimbusMotorController(MSP=MOTOR_SWITCH_PIN,
                                        MLTParams=LTParams,
                                        MLBParams=LBParams,
                                        MRTParams=RTParams,
                                        MRBParams=RBParams,
                                        timeOut=2500)


def callBackFunc(tim):
    global i2cPeripheralHandler, batteryCutoffVoltage, uart
    rxS = i2cPeripheralHandler.p_i2c.rxFifoSize()
    wI = i2cPeripheralHandler.p_i2c.anyRead()
    rI = i2cPeripheralHandler.p_i2c.any()
    """    
    if wI or rI or rxS != 0:
        # print(rxS, wI, rI)
        # uart.write(b"rxS:{0},wI:{1},rI:{2}\r\n".format(rxS, str(wI), str(rI)))
        # uart.write(bytearray("b"))

    if rxS is 0 and wI and not rI:
        # uart.write(b"rxS:{0},wI:{1},rI:{2}\r\n".format(rxS, str(wI), str(rI)))
        i2cPeripheralHandler.p_i2c.put(0x00)  # For general call release
    elif rxS is 0 and not wI and rI:
        # uart.write(b"rxS:{0},wI:{1},rI:{2}\r\n".format(rxS, str(wI), str(rI)))
        _ = i2cPeripheralHandler.p_i2c.get()  # For general call release
    """
    if rI:
        readAv = True
        reg = i2cPeripheralHandler.p_i2c.get()
        # print("R:{0}".format(reg))
        # uart.write(b"R:{0}\r\n".format(reg))
        if reg == i2cPeripheralHandler.regs.NR:
            # print("IN R")
            reg = i2cPeripheralHandler.p_i2c.get()  # Read the incoming byte so controller doesn't stay waiting.
            # print("Resetting RP2040")
            # uart.write(b"Resetting RP2040\r\n")
            time.sleep(0.01)
            machine.reset()
        if wI:
            i2cPeripheralHandler.writeToController(reg)
        else:
            i2cPeripheralHandler.readFromController(reg)
            if reg is i2cPeripheralHandler.regs.NLED:
                led_onboard.value(i2cPeripheralHandler.dataFieldRx[reg])
            elif reg is i2cPeripheralHandler.regs.NMSP:
                mc.MSP(state=i2cPeripheralHandler.dataFieldRx[reg])
            elif reg is i2cPeripheralHandler.regs.NNPXL:
                # print("Here")
                # uart.write(b"Here\r\n")
                ws2812b.pixels_set(0, (i2cPeripheralHandler.dataFieldRx[reg],
                                       i2cPeripheralHandler.dataFieldRx[reg + 1],
                                       i2cPeripheralHandler.dataFieldRx[reg + 2]))
                ws2812b.pixels_show()
                # print("Here Again")
                # uart.write(b"Here Again\r\n")
            elif reg is i2cPeripheralHandler.regs.NBCV:
                batteryCutoffVoltage = struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]
                # print("BV:{0}".format(batteryCutoffVoltage))
                # uart.write(b"BV:{0}\r\n".format(batteryCutoffVoltage))
            elif reg is i2cPeripheralHandler.regs.NMLTS:
                mc.MLTS(int(struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]))
            elif reg is i2cPeripheralHandler.regs.NMLBS:
                mc.MLBS(int(struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]))
            elif reg is i2cPeripheralHandler.regs.NMRTS:
                mc.MRTS(int(struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]))
            elif reg is i2cPeripheralHandler.regs.NMRBS:
                mc.MRBS(int(struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]))
            elif reg is i2cPeripheralHandler.regs.NMT:
                mc.MTA(int(struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]))
                # print("TO:{0}".format(mc.MTA()))
                # uart.write(b"TO:{0}\r\n".format(mc.MTA()))
            elif reg is i2cPeripheralHandler.regs.NMTVPT:
                # print("Here")
                # uart.write(b"Here\r\n")
                theta = struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg:reg + 4])[0]
                vel = struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg + 4:reg + 8])[0]
                phi = struct.unpack('f', i2cPeripheralHandler.dataFieldRx[reg + 8:reg + 12])[0]
                # print("theta:{0},vel:{1},phi:{2}".format(theta, vel, phi))
                # uart.write(b"theta:{0},vel:{1},phi:{2}\r\n".format(theta, vel, phi))
                mc.MVA(theta=theta, vel=vel, phi=phi)
            elif reg is i2cPeripheralHandler.regs.NMTB:
                mc.MTB(i2cPeripheralHandler.dataFieldRx[reg])


i2cPeripheralHandler = nimbusI2CPeripheral.nimbusI2CPeripheral(callBackFunction=callBackFunc,
                                                               initialBatteryCutoff=batteryCutoffVoltage,
                                                               frequency=100,
                                                               initialRobotTwistTime=robotTwistTime,
                                                               i2cAddress=0x45)


def looper():
    dataDict = {}
    while True:
        battLevel = adc.readBatteryLevel_V()
        boostLevel = adc.readPowerBooster_V()
        mltC = adc.readMotorLeftTopCurrent_mA()
        mlbC = adc.readMotorLeftBottomCurrent_mA()
        mrtC = adc.readMotorRightTopCurrent_mA()
        mrbC = adc.readMotorRightBottomCurrent_mA()

        dataDict['BL'] = "{0:.2f}".format(battLevel)
        dataDict['PBL'] = "{0:.2f}".format(boostLevel)
        dataDict['mltC'] = "{0:.2f}".format(mltC)
        dataDict['mlbC'] = "{0:.2f}".format(mlbC)
        dataDict['mrtC'] = "{0:.2f}".format(mrtC)
        dataDict['mrbC'] = "{0:.2f}".format(mrbC)
        dataDict['BTN'] = button.value()
        dataDict_str = str(dataDict) + "\r\n"
        uart.write(bytearray(dataDict_str))
        # i2cPeripheralHandler.fillTxFloat(i2cPeripheralHandler.regs.NBLV, battLevel)
        # i2cPeripheralHandler.fillTxFloat(i2cPeripheralHandler.regs.NPBV, boostLevel)
        # i2cPeripheralHandler.fillTxFloat(i2cPeripheralHandler.regs.NMLTC, mltC)
        # i2cPeripheralHandler.fillTxFloat(i2cPeripheralHandler.regs.NMLBC, mlbC)
        # i2cPeripheralHandler.fillTxFloat(i2cPeripheralHandler.regs.NMRTC, mrtC)
        # i2cPeripheralHandler.fillTxFloat(i2cPeripheralHandler.regs.NMRBC, mrbC)
        # i2cPeripheralHandler.dataFieldTx[i2cPeripheralHandler.regs.NBTN] = button.value()
        #
        # dataToUart = "{'BL':{0},'PBL':{1},'mltC':{2},'mlbC':{3},'mrtC':{4},'mrbC':{5},'BTN':{6}}".format(
        #     battLevel,
        #     boostLevel,
        #     mltC,
        #     mlbC,
        #     mrtC,
        #     mrbC,
        #     button.value())
        #
        # uart.write()

        time.sleep(0.5)


looper()
