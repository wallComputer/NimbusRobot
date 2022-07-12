import time
from machine import Pin
from nrpd import *
from lib import test

led = Pin(BLUELED_PIN, Pin.OUT)
print("Hello, World! Value from test.py: {Value:}".format(Value=test.testValue))

while True:
    led.toggle()
    time.sleep(1)
