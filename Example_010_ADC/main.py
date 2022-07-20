import time

from machine import Pin, I2C
from nrpd import *
from lib import adcWTMux, motors

from ssd1306 import SSD1306_I2C
from nrpd import *
import framebuf

i2c=I2C(0,sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)
mc = motors.motorControllers()
adc = adcWTMux.adcWTMux()
devices = i2c.scan()
oled.text(str(devices),0,0)
oled.show()
time.sleep(5)


while True:
    oled.fill(0)
    oled.text("BT: {BT: .2f}V".format(BT=adc.readBatteryLevel_V()), 0, 0)
    oled.text("PB: {PB:.2f}V".format(PB=adc.readPowerBooster_V()), 0, 32)
    oled.show()
    time.sleep(0.5)
# N = 4000
# currentOffsets = [0,0,0,0]
# for i in range(0,N):
#     currentOffsets[0] = currentOffsets[0]+adc.readMotorLeftTopCurrent_mA()
#     currentOffsets[1] = currentOffsets[1]+adc.readMotorLeftBottomCurrent_mA()
#     currentOffsets[2] = currentOffsets[2]+adc.readMotorRightTopCurrent_mA()
#     currentOffsets[3] = currentOffsets[3]+adc.readMotorRightBottomCurrent_mA()
#     time.sleep(0.01)
#     print(i/N*100)
#
# for i in range(0,4):
#     currentOffsets[i] = currentOffsets[i]/N
#
# mc.MSP = 1
# print("[")
# for i in range(-100,101):
#     # mc.MA = [50,50,50,50]
#     mc.MA = [i,i,i,i]
#     time.sleep(0.2)
#     print([adc.readMotorLeftTopCurrent_mA(), adc.readMotorLeftBottomCurrent_mA(),  adc.readMotorRightTopCurrent_mA(),adc.readMotorRightBottomCurrent_mA()],",")
# mc.MSP = 0
# print("]")
