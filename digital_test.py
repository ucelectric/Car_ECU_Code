import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(26, GPIO.IN)
while True:
    switch= GPIO.input(26)
    if switch==1:
        print('ON')
    else:
        print('OFF')