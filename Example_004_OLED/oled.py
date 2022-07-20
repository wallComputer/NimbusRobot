from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
from nrpd import *

i2c=I2C(0,sda=Pin(I2C_SDA), scl=Pin(I2C_SCL), freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

oled.text("Hello, World!!", 0, 0)
oled.text("Nimbus Robot", 0, 16)
oled.text("wallComputer", 0, 32)
oled.show()
