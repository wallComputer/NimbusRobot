import time
from machine import Pin
from nrpd import *

button = Pin(BUTTON_PIN, Pin.IN)

while True:
    print("Button State: {}".format(button.value()), end="\r")
    time.sleep(0.1)