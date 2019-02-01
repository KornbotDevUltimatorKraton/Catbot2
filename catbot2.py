import microgear.client as microgear # Microgear for the Internet of Thing function
import math # Math function for the robotic location control and movement 
from sklearn.linear_model import LinearRegression # Linear Regression learning function 
import sklearn.datasets # Working on the data set function 
import numpy as np 
import cv2 
import scipy
import pandas as pd # for the data function file 
import csv # the csv file writer for the data that keep on the system for learning function 
from nanpy import(ArduinoApi,SerialManager) # Serial manager function for the hardware control  
from nanpy import Servo
import serial 
import os
import sys 
import smbus # i2c interface library on the rpi 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.
       # Sensor and the Actuator parameter
 # Leg servo  
Servo1 = Servo(2)
Servo2 = Servo(3)
Servo3 = Servo(4) 
Servo4 = Servo(5)
  # body rotation servo 
Servo5 = Servo(8)
  # tail tentacle movement of machanism function 
Servo6 = Servo(9)    # X 
Servo7 = Servo(10)   # Y 
   # Analog Sensors Reader sensing  
Sensing1 = 0 
Sensing2 = 0
Sensing3 = 0
Sensing4 = 0   
   # Angle Read potentiometer
AnglefrontLeft = 0 
AngleBackLeft  = 0
AnglefrontRight = 0 
AngleBackRight = 0
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
  # Gyro scope address 
# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
 
# Serial connection 
try: 
  connection = SerialManager()  #Serial connection  
  hydraulicunit = ArduinoApi(connection=connection) # Connection estrablished function for the Hardware onboard 
except: 
  print("Hardware serial connection error")
  sys.exit(0) # Exit from the software if got the error on the hardware connection 
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    # Gyro sensor calculation function on the body dynamic

def read_byte(reg):
    return bus.read_byte_data(address, reg)
 
def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value
 
def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
 
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
 
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
 
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
     # main function and natural behavier pre-program  for the robotics cat 
        # DC pump/ BLDC pump  need for high pressure output on the hydraulic actuator control unit 
def Pumppressure(Pressure):   # Front pressure pump 
    hydraulicunit.analogWrite(11,Pressure)  # analog Write the hydraulic control functon 

def Pumphighpressure(Pressure): # Back pressure pump  
    hydraulicunit.analogWrite(12,Pressure)  

def hydraulicFrontlegRight(goal,angle,pressure):
   if goal >  angle:
       hydraulicunit.digitalWrite(23,hydraulicunit.LOW)  # Hydraulic unit valve go high
       hydraulicunit.digitalWrite(25,hydraulicunit.HIGH) # Hydraulic unit valve go low  
       Pumppressure(pressure)  # Pressure determine from the even needed for agility on the robot funtion system onboard 
   if goal == angle: 
       hydraulicunit.digitalWrite(23,hydraulicunit.HIGH)
       hydraulicunit.digitalWrite(25,hydraulicunit.LOW) 
       Pumppressure(0) # turn off the pump when the angle at the right position 
   if goal < angle: 
       hydraulicunit.digitalWrite(23,hydraulicunit.LOW)
       hydraulicunit.digitalWrite(25,hydraulicunit.HIGH)
       Pumppressure(pressure) 
def hydraulicFrontlegLeft(goal,angle,Pumppressure):
    if goal >  angle:
       hydraulicunit.digitalWrite(27,hydraulicunit.LOW)  # Hydraulic unit valve go high
       hydraulicunit.digitalWrite(29,hydraulicunit.HIGH) # Hydraulic unit valve go low  
       Pumppressure(Pumphighpressure)  # Pressure determine from the even needed for agility on the robot funtion system onboard 
    if goal == angle: 
       hydraulicunit.digitalWrite(27,hydraulicunit.HIGH)
       hydraulicunit.digitalWrite(29,hydraulicunit.LOW) 
       Pumppressure(0) # turn off the pump when the angle at the right position 
    if goal < angle: 
       hydraulicunit.digitalWrite(27,hydraulicunit.LOW)
       hydraulicunit.digitalWrite(29,hydraulicunit.HIGH)
       Pumppressure(Pumphighpressure) 
def hydraulicBacklegRight(goal,angle,pressure): 
   if goal >  angle:
       hydraulicunit.digitalWrite(31,hydraulicunit.LOW)  # Hydraulic unit valve go high
       hydraulicunit.digitalWrite(32,hydraulicunit.HIGH) # Hydraulic unit valve go low  
       Pumphighpressure(pressure)  # Pressure determine from the even needed for agility on the robot funtion system onboard 
   if goal == angle: 
       hydraulicunit.digitalWrite(31,hydraulicunit.HIGH)
       hydraulicunit.digitalWrite(32,hydraulicunit.LOW) 
       Pumphighpressure(0) # turn off the pump when the angle at the right position 
   if goal < angle: 
       hydraulicunit.digitalWrite(31,hydraulicunit.LOW)
       hydraulicunit.digitalWrite(32,hydraulicunit.HIGH)
       Pumphighpressure(pressure)
def hydraulicfBacklegLeft(goal,angle,pressure): 
   if goal >  angle:
       hydraulicunit.digitalWrite(28,hydraulicunit.LOW)  # Hydraulic unit valve go high
       hydraulicunit.digitalWrite(30,hydraulicunit.HIGH) # Hydraulic unit valve go low  
       Pumphighpressure(pressure)  # Pressure determine from the even needed for agility on the robot funtion system onboard 
   if goal == angle: 
       hydraulicunit.digitalWrite(28,hydraulicunit.HIGH)
       hydraulicunit.digitalWrite(30,hydraulicunit.LOW) 
       Pumphighpressure(0) # turn off the pump when the angle at the right position 
   if goal < angle: 
       hydraulicunit.digitalWrite(28,hydraulicunit.LOW)
       hydraulicunit.digitalWrite(30,hydraulicunit.HIGH)
       Pumphighpressure(pressure) 

    # Function of the leg sequence for the angle movement and realistic behavier function 
def ServolegSequence(angle1,angle2,angle3,angle4,angle5):
    Servo1.write(angle1)
    Servo2.write(angle2)
    Servo3.write(angle3)
    Servo4.write(angle4)
    Servo5.write(angle5)
    #tail dynamics control function for the robotic dynamics movement 
def Servotailmovement(angle6,angle7): 
    Servo6.write(angle6)
    Servo7.write(angle7) 
    # Sequence and pressure actuator control function 
def hydraulicsequence(goal1,goal2,goal3,goal4,Angle1,Angle2,Angle3,Angle4,pressure): 
     # Pressure and angle control function for the hydraulic system 
    hydraulicFrontlegLeft(goal1,Angle1,pressure)
    hydraulicFrontlegRight(goal2,Angle2,pressure) 
    hydraulicBacklegRight(goal3,Angle3,pressure)
    hydraulicfBacklegLeft(goal4,Angle4,pressure)

while True: 
           #All Angle and sensor read for each condition Leg function 
        AnglefrontLeft = hydraulicunit.analogRead(0)
        AngleBackRight = hydraulicunit.analogRead(1) 
        AnglefrontRight = hydraulicunit.analogRead(2)
        AngleBackRight = hydraulicunit.analogRead(3) 
        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          #Front head control sensor function 
        Sensing1 = hydraulicunit.analogRead(4)
          #Front body  
        Sensing2 = hydraulicunit.analogRead(5)
          #Back body 
        Sensing3 = hydraulicunit.analogRead(6)
         # tail for the movment actiion 
        Sensing4 = hydraulicunit.analogRead(7)
        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
          #Gyroscope sensor 
        bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
        address = 0x68       # via i2cdetect
 
         # Aktivieren, um das Modul ansprechen zu koennen
        bus.write_byte_data(address, power_mgmt_1, 0)
 
        print "Gyroskop"
        print "--------"
        gyroskop_xout = read_word_2c(0x43)
        gyroskop_yout = read_word_2c(0x45)
        gyroskop_zout = read_word_2c(0x47)
 
        print "gyroskop_xout: ", ("%5d" % gyroskop_xout), " skaliert: ", (gyroskop_xout / 131)
        print "gyroskop_yout: ", ("%5d" % gyroskop_yout), " skaliert: ", (gyroskop_yout / 131)
        print "gyroskop_zout: ", ("%5d" % gyroskop_zout), " skaliert: ", (gyroskop_zout / 131)
        print "Beschleunigungssensor"
        print "---------------------"
        beschleunigung_xout = read_word_2c(0x3b)
        beschleunigung_yout = read_word_2c(0x3d)
        beschleunigung_zout = read_word_2c(0x3f)
        beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
        beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
        beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
        print "beschleunigung_xout: ", ("%6d" % beschleunigung_xout), " skaliert: ", beschleunigung_xout_skaliert
        print "beschleunigung_yout: ", ("%6d" % beschleunigung_yout), " skaliert: ", beschleunigung_yout_skaliert
        print "beschleunigung_zout: ", ("%6d" % beschleunigung_zout), " skaliert: ", beschleunigung_zout_skaliert
        print "X Rotation: " , get_x_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
        print "Y Rotation: " , get_y_rotation(beschleunigung_xout_skaliert, beschleunigung_yout_skaliert, beschleunigung_zout_skaliert)
         #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>