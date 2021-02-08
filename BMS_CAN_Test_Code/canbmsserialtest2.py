import time
import serial

"""ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    bytesize=serial.EIGHTBITS,
    timeout=2)

while True:
    x=ser.readline()
    print(x)"""
def openbms(port= '/dev/ttyUSB0'):
    ser = serial.Serial(port)
    ser.timeout = 2
    return ser.readline()
print(openbms())