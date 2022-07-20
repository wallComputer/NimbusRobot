# import time
# from machine import UART, Pin
# from nrpd import *
# uart = UART(0, baudrate=9600, tx=Pin(UART_TX), rx=Pin(UART_RX))
#
# size = uart.write("Hello, World!\r\n")
#
# time.sleep(5)
# print(uart.read(10))