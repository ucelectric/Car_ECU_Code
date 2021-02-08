import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(26,GPIO.OUT)

print('OFF')
GPIO.output(26, GPIO.LOW)
time.sleep(5)

print('ON')
GPIO.output(26, GPIO.HIGH)
time.sleep(5)

print('OFF')
GPIO.output(26, GPIO.LOW)
time.sleep(5)
GPIO.cleanup()