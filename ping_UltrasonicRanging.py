#!/usr/bin/python


import RPi.GPIO as GPIO

from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD

import time

MAX_DISTANCE = 220          # define the maximum measuring distance, unit: cm
timeOut = MAX_DISTANCE*60   # calculate timeout according to the maximum measuring distance
trigPin=11

def pulseIn(pin,level,timeout): # obtain pulse time of a pin under timeOut
    t0 = time.time()
    while(GPIO.input(pin) != level):
        if((time.time() - t0) > timeout*0.000001):
            return 0;
    t0 = time.time()
    while(GPIO.input(pin) == level):
        if((time.time() - t0) > timeout*0.000001):
            return 0;
    pulseTime = (time.time() - t0)*1000000
    return pulseTime

def microsecondsToInches(microseconds):
    '''
      According to Parallax's datasheet for the PING))), there are 73.746
      microseconds per inch (i.e. sound travels at 1130 feet per second).
      This gives the distance travelled by the ping, outbound and return,
      so we divide by 2 to get the distance of the obstacle.
      See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
    '''
    return microseconds / 74 / 2

def microsecondsToCentimeters(microseconds):
    '''
       The speed of sound is 340 m/s or 29 microseconds per centimeter.
       The ping travels out and back, so to find the distance of the object we
       take half of the distance travelled.
    '''
    return microseconds / 29 / 2


def destroy():
     GPIO.cleanup()
     mcp.output(3,0)   #turn off LCD backlight


def loop():
    global mcp
    GPIO.setmode(GPIO.BOARD)
    PCF8574_address = 0x27  # I2C address of the PCF8574 chip.
    PCF8574A_address = 0x3F  # I2C address of the PCF8574A chip.
    # Create PCF8574 GPIO adapter.
    try:
        mcp = PCF8574_GPIO(PCF8574_address)
    except:
        try:
            mcp = PCF8574_GPIO(PCF8574A_address)
        except:
            print ('I2C Address Error !')
            exit(1)
    # Create LCD, passing in MCP GPIO adapter.
    lcd = Adafruit_CharLCD(pin_rs=0, pin_e=2, pins_db=[4,5,6,7], GPIO=mcp)
    mcp.output(3,1)     # turn on LCD backlight
    lcd.begin(16,2)     # set number of LCD lines and columns
    lcd.clear()

    while (True):
        
        lcd.setCursor(0,0)  # set cursor position

        #   pinMode(pingPin, OUTPUT);
        #   digitalWrite(pingPin, LOW);
        #   delayMicroseconds(2);
        GPIO.setup(trigPin, GPIO.OUT)
        GPIO.output(trigPin, GPIO.LOW)
        time.sleep(0.000002)

        #   digitalWrite(pingPin, HIGH);
        #   delayMicroseconds(5);
        GPIO.output(trigPin, GPIO.HIGH)
        time.sleep(0.000005)

        #    pinMode(pingPin, INPUT);
        GPIO.setup(trigPin, GPIO.IN)

        pingTime = pulseIn(trigPin,GPIO.HIGH,timeOut)   # read pulse time

        print ('cm:%s'%microsecondsToCentimeters(pingTime),end ='\t')
        print ('in:%s'%microsecondsToInches(pingTime),end='\t')
        print ('ms:%s'%pingTime)
        lcd.message('cm:' + str(microsecondsToCentimeters(pingTime))[:4]+ '\n')
        lcd.message('in:' + str(microsecondsToInches(pingTime))[:4])

        time.sleep(.2)

if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
             
        loop()
    except KeyboardInterrupt:
        destroy()