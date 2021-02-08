import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import serial
import board
import digitalio
import busio
import adafruit_lis3dh

### Initial setup of gpio
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

### Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
ser=serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=10
)

#### Accelerometer/ I2C Cofiguration
if hasattr(board, 'ACCELEROMETER_SCL'): #figure out which choice it uses and then erase the other one
    i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
    int1 = digitalio.DigitalInOut(board.ACCELEROMETER_INTERRUPT)
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address=0X19, int1=int1)
else:
    i2c = busio.I2C(board.SCL, board.SDA)
    int1 = digitalio.DigitalInOut(board.D6) #set to correct pin for interrupt
    lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, int1=int1)
#set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G, or RANGE_16_G)
lis3dh.range = adafruit_lis3dh.RANGE_4_G

######## Variables/Pins
Shutdown_Circuit_Input=4#? #HIGH is tractive system is on. might not need/want this as might just be able to use voltage value
Start_Button_Input=4 #high is on
RTDS_Output=24
MC_Switch=26
Brake_Light_Output=23
BPPC_LED_Output=25
Boolean_to_prevent_RTDS_More=True
Boolean_to_help_BPPC=True
#Analog Inputs
Battery_Percentage_Input=2 #2 of the analog ADC converter
APPS_Input=3 #3 of the analog ADC converter
Brake_Pedal_Input=4 #4 of the analog ADC converter
Battery_Voltage_Input=5 #5 of the analog ADC converter


######## Setup
GPIO.setup(Shutdown_Circuit_Input, GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(Start_Button_Input, GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(RTDS_Input, GPIO.OUT)
GPIO.setup(MC_Switch, GPIO.OUT)
GPIO.setup(Brake_Light_Output, GPIO.OUT)
GPIO.setup(BPPC_LED_Output, GPIO.OUT)

###### MAIN LOOP
while True:
    ### Need to start like this
    GPIO.output(MC_Switch, 0) #needs to start low
    
    ### Status Variables 
    Shutdown_Circuit_Status = GPIO.input(Shutdown_Circuit_Input)
    Brake_Pedal_Percentage_Status = (mcp.read_adc(Brake_Pedal_Input-1))*(100/1023)
    Battery_Voltage_Status = (mcp.read_adc(Battery_Voltage_Input-1))*((167.5*5)/1023)
    APPS_Percentage_Status = (mcp.read_adc(APPS_Input-1))*(100/1023)
    Battery_Percentage_Status = (mcp.read_adc(Battery_Percentage_Input-1))*(100/1023)
    Start_Button_Status = GPIO.input(Start_Button_Input)
    
    #Read Accelerometer
    #read values (in m /s^2).
    #returns a 3-tuple of x, y, z axis values. divide by 9.806 to convert to G's
    x_accel, y_accel, z_accel = [value / adafruit_lis3dh.STANDARD_GRAVITY for value in lis3dh.acceleration]

    ### Ready to Drive Procedure:
    if Boolean_to_prevent_RTDS_More and Shutdown_Circuit_Status and Start_Button_Status and Brake_Pedal_Percentage_Status > 20:
        GPIO.output(MC_Switch, 1)
        GPIO.output(RTDS_Output, 1)
        time.sleep(2)
        GPIO.output(RTDS_Output, 0)
        Boolean_to_prevent_RTDS_More= False
    
    ###Brake light control
    if Brake_Pedal_Percentage_Status > 5: #could change from 5
        GPIO.output(Brake_Light_Output, 1)
    else:
        GPIO.output(Brake_Light_Output, 0)
        
    ###Brake Pedal Plausibility Check (BPPC)
    if Brake_Pedal_Percentage_Status > 10 and APPS_Percentage_Status > 25 and Boolean_to_help_BPPC==True: #could change from 10
        GPIO.output(MC_Switch, 0)
        GPIO.output(BPPC_LED_Output, 1)
        Boolean_to_help_BPPC=False
    #To reverse BPPC- might be able to just make this an else statement
    if Boolean_to_help_BPPC==False and APPS_Percentage_Status < 5:
        GPIO.output(MC_Switch, 1)
        GPIO.output(BPPC_LED_Output, 0)
        Boolean_to_help_BPPC=True
    
        
        
    
