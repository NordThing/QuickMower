#@author Henrik Allberg
#This is my autopilot trying to clean up some code and follow a flow-chart to get it right direct

#Start with importing all packages that are needed
import os
from gps import *
import string
from pynmea import nmea
import socket
import sys
import RPi.GPIO as GPIO
from time import sleep
import numpy as np
import hmc5883l as HMC
import gpscalculatemodule as GPSCalc

#Initialise to get working with GPIO and disable warnings
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

#Calibration constants for the motors in differential drive config.
# e. g. if the left motor is slower that the right one, you can add some 
# % to the left and reduce the same % to the right, and viceversa.#Initialise motor 1 and motor 2 on controller L298
K_RIGHT_MOTOR = 1.0
K_LEFT_MOTOR = 1.0
vel = 255

#Rover velocities (rango of allowe velocities 0 - 255)
FORWARD_VEL = 230
SLOW_TURN_VEL = 220
PIVOT_WHEEL_VEL = 50
FAST_TURN_VEL = 180

#Constants for defining error ranges for the stepped proportinal control
MAX_HEADING_ANGLE = 180
MIN_HEADING_ANGLE = 5
ANGLE_RANGE_DIV = 0.25

#Motor 1
Motor1A = 16
Motor1B = 18
Motor1E = 32
GPIO.setup(Motor1A,GPIO.OUT)
GPIO.setup(Motor1B,GPIO.OUT)
GPIO.setup(Motor1E,GPIO.OUT)
pwmMotor1 = GPIO.PWM(Motor1E, vel * K_RIGHT_MOTOR)

#Motor 2
Motor2A = 21
Motor2B = 23
Motor2E = 33
GPIO.setup(Motor2A,GPIO.OUT)
GPIO.setup(Motor2B,GPIO.OUT)
GPIO.setup(Motor2E,GPIO.OUT)
pwmMotor2 = GPIO.PWM(Motor2E, vel * K_LEFT_MOTOR)

#Set parameters that are needed to get everything working

#Tolerance how near you need to be the waypoint to set it as reached higher value less sensitive
TOLERANCE_RADIUS = 1.0
#Index of waypoints 
waypoint_index = 0
#Initialistion for the calculated distance
#and for the calculated course angle just to have variables
act_distance = 0
act_angle = 0
calc_distance = 0
calc_angle = 0

# Create an instance of an GPGGA object.
gpgga = nmea.GPGGA()

#Here is the waypoint that the program will use
nav_waypoint = np.loadtxt("waypoints.txt", dtype=np.float16, delimiter=",", skiprows=1)
#nav_waypoint = np.array([[1,2],[3,4],[5,6]])
#Number of waypoints to use in the loop so all waypoints will be went trough
num_waypoints = (len(nav_waypoint))

#Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Define the server address and port from which the RTKLIB NMEA
# should be read from.
server_address = ('localhost', 50022)

try:
        # Connect the socket to the port where the RTK server is listening
        sock.connect(server_address)
        print '-------------------------------------------'
        print >>sys.stderr, 'connecting to <%s> port <%s>' % server_address
        print '-------------------------------------------'
except:
    print "Unexpected error:", sys.exc_info()[0]
    raise

# Read each line from the NMEA stream.
def readlines(sock, recv_buffer=4096, delim='\n'):
        buffer = ''
        data = True
        while data:
                data = sock.recv(recv_buffer)
                buffer += data

                while buffer.find(delim) != -1:
                        line, buffer = buffer.split('\n', 1)
                        yield line
        return

#Setting global variable for waypoint_lat and waypoint_lon to get acces to the globals outside the method
def act_waypoint():
  print 'Actual waypoint we are heading against'
  print(nav_waypoint[0,0:2])
  global waypoint_lat
  global waypoint_lon
  waypoint_lat = nav_waypoint[0,0]
  waypoint_lon = nav_waypoint[0,1]

#Here is all the coding for the PWM-control for the motors
#Moving the rover forward
def Forward():
  GPIO.output(Motor1A,GPIO.HIGH)
  GPIO.output(Motor1B,GPIO.LOW)
  GPIO.output(Motor1E,GPIO.HIGH)
  GPIO.output(Motor2A,GPIO.HIGH)
  GPIO.output(Motor2B,GPIO.LOW)
  GPIO.output(Motor2E,GPIO.HIGH)

def Turn_Left(vel):
  pwmMotor1 = GPIO.PWM(Motor1E, vel * K_RIGHT_MOTOR)
  pwmMotor2 = GPIO.PWM(Motor2E, PIVOT_WHEEL_VEL * K_LEFT_MOTOR)

def Turn_Right(vel):
  pwmMotor1 = GPIO.PWM(Motor1E, PIVOT_WHEEL_VEL * K_RIGHT_MOTOR)
  pwmMotor2 = GPIO.PWM(Motor2E, vel * K_LEFT_MOTOR)

def Turn_Left_Fast_Spin(vel):
  pwmMotor1 = GPIO.PWM(Motor1E, vel * K_RIGHT_MOTOR)
  pwmMotor2 = GPIO.PWM(Motor2E, vel * K_LEFT_MOTOR)

def Turn_Right_Fast_Spin():
  pwmMotor1 = GPIO.PWM(Motor1E, vel * K_RIGHT_MOTOR)
  pwmMotor2 = GPIO.PWM(Motor2E, vel * K_LEFT_MOTOR)

#Stopping all motors on rover
def Stop():
  GPIO.output(Motor1E,GPIO.LOW)
  GPIO.output(Motor2E,GPIO.LOW)

def Gps():
# Process the NMEA data and store it in two variables continuous.
  try:
        for line in readlines(sock):
              #print line
              if line[0:6] == '$GPGGA':
                     os.system('clear')
  
                     #method for parsing the NMEA sentence
                     gpgga.parse(line)
                     #Latitude direction       
                     lat_dir = gpgga.lat_direction

                     #Longitude values
                     longitude = gpgga.longitude

                     #Longitude direction
                     long_dir = gpgga.lon_direction
                       
                     #GPS time stamps
                     time_stamp = gpgga.timestamp
                        
                     #Antenna altitude
                     alt = gpgga.antenna_altitude
                        
                     lats = gpgga.latitude
                     longs = gpgga.longitude

                     #convert degrees,decimal minutes to decimal degrees
                     # The source for the decimal calcualtion was:
                     # http://dlnmh9ip6v2uc.cloudfront.net/tutorialimages/Python_and_GPS/gpsmap.py
                     lat1 = (float(lats[2]+lats[3]+lats[4]+lats[5]+lats[6]+lats[7]+lats[8]))/60
                     lat = (float(lats[0]+lats[1])+lat1)
                     long1 = (float(longs[3]+longs[4]+longs[5]+longs[6]+longs[7]+longs[8]+longs[9]))/60
                     long = (float(longs[0]+longs[1]+longs[2])+long1)

                     print '\n------------ rover information --------------'
                     print 'lat: ',lat
                     print 'long: ',long
                     print 'direction: ', HMC.bearing()
                     print '-------------------------------------------'
  except:
         print "Unexpected error:", sys.exc_info()[0]
         raise

def Compute_Navigation_Vector():
  distance, angle = GPSCalc.calc(lat, long, waypoint_lat, waypoint_lon)

act_waypoint()
HMC.bearing()
Gps()
act_waypoint()
