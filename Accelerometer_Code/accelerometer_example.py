import time
import board
import digitalio
import busio
import adafruit_lis3dh

if hasattr(board, 'ACCELEROMETER_SCL'):
    i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
    int1 = digitalio.DigitalInOut(board.ACCELEROMETER_INTERRUPT)
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address=0X19, int1=int1)
else:
    i2c = busio.I2C(board.SCL, board.SDA)
    int1 = digitalio.DigitalInOut(board.D6) #set to correct pin for interrupt
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, int1=int1)
    
#set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G, or RANGE_16_G)
lis3dh.range = adafruit_lis3dh.RANGE_4_G

#loop forever, printing accelerometer values
while True:
    #read values (in m /s^2).
    #returns a 3-tuple of x, y, z axis values. divide by 9.806 to convert to G's
    x, y, z = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]
    print("x = %0.3f G, y = %0.3f G, z = %0.3f G" % (x, y, z))
    #small delay to keep things responsive but give time for interrupt processing
    time.sleep(0.1)