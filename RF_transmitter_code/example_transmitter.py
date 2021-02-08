import serial
import time
import RPi.GPIO as GPIO
ser=serial.Serial(port="/dev/ttyS0",
                  baudrate=9600,
                  parity=serial.PARITY_NONE,
                  stopbits=serial.STOPBITS_ONE,
                  bytesize=serial.EIGHTBITS,
                  timeout=1)
counter=0
while True:
    ser.write(str.encode('%d \n' %(counter)))
    time.sleep(1)
    print(counter)
    counter+=1