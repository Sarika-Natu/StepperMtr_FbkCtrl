#SID: 01452 - 9138 
#Course: EE242
#Name: Sarika Satish Natu

import pigpio
import time, sys
import spidev
from time import sleep
import os
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math

stp = 20                                   #GPIO_20
dir = 21                                   #GPIO_21
Boardpin = 23                              #GPIO_23
pot_channel = 0                            #ADC channel 
sleepTime = 1

spi = spidev.SpiDev()                      #setup SPI bus for communicating with MCP3008 ADC
spi.open(0,0)
spi.max_speed_hz=1000000

i2c = busio.I2C(board.SCL, board.SDA)      #setup I2C configuration
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c) #read LMS303 magnetomoter data


pi = pigpio.pi()                           #setup dir and step as output  for Easy driver                
pi.set_mode(dir, pigpio.OUTPUT)
pi.set_mode(stp, pigpio.OUTPUT)

def move(duty, direction):                 #stepping motor motion function
    pi.write(dir, direction)
    pi.set_PWM_dutycycle(stp, duty)
    pi.set_PWM_frequency(stp, 500)
    sleep(.05) 

def readADCch(channel):                    #Read ADC data from MCP008
    adc = spi.xfer2([1,(8+channel)<<4,0])
    data = ((adc[1]&3) << 8) + adc[2]
    return data

def convertDuty(angle):                    #Convert target angle into duty cycle
    duty = (angle) / float(360)
    duty = duty * 255 #pwm full cycle
    duty = int(duty)
    return duty
 
def convertVoltage(bitValue):              #Convert ADC data into voltage
    voltage = (bitValue * 3.3) / float(1023)
    voltage = round(voltage, 2)
    return voltage

def convertAngle(vtg):                     #Convert ADC voltage into angular form(degrees)   
    angle = (vtg) / float(3.3)
    angle = angle * 360 
    angle = int(angle)
    return angle

def getHeading(mag_x, mag_y):              #Convert LMS303 sensor output micro-Teslas into degrees
        heading = math.atan2(mag_x, mag_y)
        if heading < 0:
            heading += 2*math.pi
        if heading > 2*math.pi:
            heading -= 2*math.pi
        headingDegrees = round(math.degrees(heading),2)
        return headingDegrees

while True:
    bitData = readADCch(pot_channel)        #get ADC data
    vtgData = convertVoltage(bitData)       #convert ADC data into voltage
    angleDataAdc = convertAngle(vtgData)    #convert ADC voltage into degrees
    
    mag_x, mag_y, mag_z = mag.magnetic      #read data from LMS303 magnetic sensor
    angleDataSen = getHeading(mag_x, mag_y) #convert sensor data into degrees
    
    angleDiff = abs(angleDataAdc - angleDataSen)
    
    print("ADC Angle={0:5.2f}Deg".format(angleDataAdc))
    print("Sensor Angle={0:5.2f}Deg".format(angleDataSen))
    print("Diff Angle={0:5.2f}Deg".format(angleDiff))
    
    dutyData = convertDuty(angleDiff)       #convert angle into duty cycle to generate PWM signal
    if (angleDataAdc > angleDataSen):
            sleep(sleepTime)
            move(dutyData, 1)               #move stepper motor in clockwise direction
    elif (angleDataAdc < angleDataSen):
            sleep(sleepTime)
            move(dutyData, 0)               #move stepper motor in anti-clockwise direction
    else: print("else block")
    
    
    print("Magnetometer: X={0:7.3f}uT Y={0:7.3f}uT Z={0:7.3f}uT".format(mag_x, mag_y, mag_z))
    print("")

