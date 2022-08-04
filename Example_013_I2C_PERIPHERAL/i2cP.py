
import machine
import time

from lib import adcWTMux, motorHandler, ws2812b
from nrpd import *

from lib.nimbusI2CPeripheral import nimbusI2CPeripheral, continuousToList2Converter

led_onboard = machine.Pin(19, machine.Pin.OUT)
button = machine.Pin(BUTTON_PIN, machine.Pin.IN)
adc = adcWTMux.adcWTMux()
batteryCutoffVoltage = 3.8
robotTwistSpeed = 0
robotTwistAngle = 0
robotTwistTime = 1500



LTParams = {
    'motorID': 0,
    'MPINS': [MLTIN1, MLTIN2],
    'motorFreq': 75,
    'sm_no': 0,
    'EncoderPin1': ELTA,
    'flipEncoder': True,
    'usePID_Distance': True,
    'PIDParams_Distance': [80, 0, 7],
    'PIDSamplingFrequency': 100
}
LBParams = {
    'motorID': 1,
    'MPINS': [MLBIN1, MLBIN2],
    'motorFreq': 75,
    'sm_no': 1,
    'EncoderPin1': ELBA,
    'flipEncoder': True,
    'usePID_Distance': True,
    'PIDParams_Distance': [50, 0, 5],
    'PIDSamplingFrequency': 100  # Hz
}
RTParams = {
    'motorID': 2,
    'MPINS': [MRTIN1, MRTIN2],
    'motorFreq': 75,
    'sm_no': 2,
    'EncoderPin1': ERTA,
    'flipEncoder': False,
    'usePID_Distance': True,
    'PIDParams_Distance': [80, 0, 5],
    'PIDSamplingFrequency': 100
}
RBParams = {
    'motorID': 3,
    'MPINS': [MRBIN1, MRBIN2],
    'motorFreq': 75,
    'sm_no': 3,
    'EncoderPin1': ERBB,
    'flipEncoder': True,
    'usePID_Distance': True,
    'PIDParams_Distance': [80, 0, 6],
    'PIDSamplingFrequency': 100
}

mc = motorHandler.nimbusMotorController(MSP=MOTOR_SWITCH_PIN,
                                        MLTParams=LTParams,
                                        MLBParams=LBParams,
                                        MRTParams=RTParams,
                                        MRBParams=RBParams)


npxl = ws2812b

def callBackFunc(tim):
    global i2cPeripheralHandler, batteryCutoffVoltage
    rxS = i2cPeripheralHandler.p_i2c.rxFifoSize()
    wI = i2cPeripheralHandler.p_i2c.anyRead()
    rI = i2cPeripheralHandler.p_i2c.any()
    if wI or rI:
        print(rxS, wI, rI)
    if rxS is 0 and wI and not rI:
        i2cPeripheralHandler.p_i2c.put(0x00)  # For general call release
    if rI:
        readAv = True
        reg = i2cPeripheralHandler.p_i2c.get()
        if reg == i2cPeripheralHandler.regs.NR:
            reg = i2cPeripheralHandler.p_i2c.get()  # Read the incoming byte so controller doesn't stay waiting.
            machine.reset()
        if wI:
            i2cPeripheralHandler.writeToController(reg)
        else:
            i2cPeripheralHandler.readFromController(reg)
            if reg is i2cPeripheralHandler.regs.NLED:
                led_onboard.value(i2cPeripheralHandler.dataFieldRx[reg])
            if reg is i2cPeripheralHandler.regs.NMSP:
                mc.MSP(state=i2cPeripheralHandler.dataFieldRx[reg])
            if reg is i2cPeripheralHandler.regs.NNPXL:
                ws2812b.pixels_set(0, (i2cPeripheralHandler.dataFieldRx[reg],
                                       i2cPeripheralHandler.dataFieldRx[reg+1],
                                       i2cPeripheralHandler.dataFieldRx[reg+2]))


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
        i2cPeripheralHandler.dataFieldTx[blPos:blPos+2] = bytearray(dataB)

        dataP = continuousToList2Converter(boostLevel, 100)
        boostLPos = i2cPeripheralHandler.regs.NPBV
        i2cPeripheralHandler.dataFieldTx[boostLPos:boostLPos+2] = bytearray(dataP)

        datamlt = continuousToList2Converter(mltC, 1)
        mltPos = i2cPeripheralHandler.regs.NMLTC
        i2cPeripheralHandler.dataFieldTx[mltPos:mltPos+2] = bytearray(datamlt)

        datamlb = continuousToList2Converter(mlbC, 1)
        mlbPos = i2cPeripheralHandler.regs.NMLBC
        i2cPeripheralHandler.dataFieldTx[mlbPos:mlbPos+2] = bytearray(datamlb)

        datamrt = continuousToList2Converter(mrtC, 1)
        mrtPos = i2cPeripheralHandler.regs.NMRTC
        i2cPeripheralHandler.dataFieldTx[mrtPos:mrtPos+2] = bytearray(datamrt)

        datamrb = continuousToList2Converter(mrbC, 1)
        mrbPos = i2cPeripheralHandler.regs.NMRBC
        i2cPeripheralHandler.dataFieldTx[mrbPos:mrbPos+2] = bytearray(datamrb)

        btnPos = i2cPeripheralHandler.regs.NBTN
        i2cPeripheralHandler.dataFieldTx[btnPos] = button.value()

        time.sleep(0.5)


looper()
