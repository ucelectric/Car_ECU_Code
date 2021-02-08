import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(False)

GPIO.setup(26, GPIO.OUT)

#creates the square wave for the muzzer with a frequency of 2048Hz
# get 0.00024 sec b/c (1/2048)/2 = 0.000244
def beep(repeat):
    for i in range(0,repeat):
        for pulse in range(60): #square wave loop
            GPIO.output(26, GPIO.HIGH)
            #time.sleep(0.008)
            time.sleep(0.00024)
            GPIO.output(26, GPIO.LOW)
            time.sleep(0.00024)
            #time.sleep(0.001)
#GPIO.output(26,GPIO.HIGH)
#time.sleep(10)
beep(220)