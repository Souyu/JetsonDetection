# coding: utf-8

import serial
import cv2
import numpy as np
import time
import dronekit as dk
from pymavlink import mavutil
import os
import math

from datetime import datetime


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dk.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    time.sleep(8)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def goto_position_target_local_ned(north, east, down): #THIS FUNCTION IS NOT NEEDED IN THIS SCRIPT
#IT IS HERE FOR REFERENCE
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
#----------------------------------------------KEEP IT THERE FOR REFERENCE
#connection_string = '/dev/ttyACM0'	#Establishing Connection With Flight Controller
#vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)
#cmds = vehicle.commands
#cmds.download()
#cmds.wait_ready()
#waypoint1 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, 3)  # Destination point 1
#----------------------------------------------

# calculate the distance from waypoint
def distance_to_waypoint (clocation, nwaypoint): # arguments are current location and waypoint
    R = 6371000 #radius of Earth in meters
    x = math.pi*(nwaypoint[1]-clocation[1])/180*math.cos(math.pi/180*(clocation[0]+nwaypoint[0])/2)
    X = R*x
    y = math.pi/180*(nwaypoint[0]-clocation[0])
    Y = R*y
    return math.sqrt(X**2+Y**2)

# reads waypoints from mission planner
def readmission(aFileName):
    print ("Reading mission from file: %s\n" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for k, line in enumerate(f):
            if k==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                #ln_autocontinue=int(linearray[10].strip())
                cmd = dk.Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, 1,\
                                  ln_param1,ln_param2, ln_param3, ln_param4, ln_param5,\
                                  ln_param6, ln_param7) #ln_autocontinue,
                missionlist.append(cmd)
    return missionlist

# allows reading the serial fully and avoid missing any bytes. *** MUST USE \n AT THE END OF LINE ***
def serialread():
    t_end = time.time() + 4             #set timer of 4 seconds
    time.sleep(.001)                    # delay of 1ms
    val = ser.readline()                # read complete line from serial output
    while not '\\n'in str(val):         # check if full data is received. 
        # This loop is entered only if serial read value doesn't contain \n
        # which indicates end of a sentence. 
        # str(val) - val is byte where string operation to check `\\n` 
        # can't be performed
        time.sleep(.001)                # delay of 1ms 
        temp = ser.readline()           # check for serial output.
        if not not temp.decode():       # if temp is not empty.
            val = (val.decode()+temp.decode()).encode()
            # requrired to decode, sum, then encode because
            # long values might require multiple passes
        elif time.time() < t_end:       # break loop if alloted time for serial reading is passed.
            break
    val = val.decode()                  # decoding from bytes
    val = val.strip()                   # stripping leading and trailing spaces.
    return val

#END of definitions!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

import jetson.inference
import jetson.utils

import argparse
import sys

input = jetson.utils.videoSource()
#output = jetson.utils.videoOutput("rtp://192.168.79.51:1234") # use for remote viewing, else comment out

#record to avi with timestamp
dir_original ='/home/souyu/Desktop/TargetRecRecording'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
output = jetson.utils.videoOutput(os.path.join(dir_original, time_stamp + '.avi'))

# load the object detection network
net = jetson.inference.detectNet()

#setting up xbee communication
#GPIO.setwarnings(False)
ser = serial.Serial(
    
    port='/dev/ttyUSB0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1   
)

# Before initializing, wait for a press of a button
print("Now waiting for Button Press...")
btn="off"
while btn !="on": #if incoming bytes are waiting to be read from the serial input buffer
    btn=serialread()

# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
#vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)# PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB
parser= argparse.ArgumentParser(description= 'commands')
parser.add_argument('--connect')
args=parser.parse_args()
connection_string= args.connect

print("Connection to the vehicle on %s" %connection_string)

vehicle= dk.connect(connection_string, wait_ready=True)

# uploading mission waypoint from text
missionwp = readmission('Spadra_Farm_Search_WPS.txt')
import_mission_filename= 'Spadra_Farm_Search_WPS.txt'

print ("\nUpload mission from a file: %s" % import_mission_filename)
# Clear existing mission from vehicle
print ("Clear mission")
cmds = vehicle.commands
cmds.clear()
# Add new mission to vehicle
for command in missionwp:
    cmds.add(command)
print ("Upload mission")
cmds.upload()
print(import_mission_filename)

cmds = vehicle.commands
cmds.wait_ready()

# Code for Flight 
arm_and_takeoff(cmds[0].z)
vehicle.parameters['WPNAV_RADIUS'] = 100
vehicle.parameters['WPNAV_SPEED'] = 200 #sets horizontal speed to 200 cm/s

vehicle.mode = dk.VehicleMode("AUTO") 

#----------------------------------------------

# establish waypoint distance
distance_wp = 5

# run image detection with confidence of 30% to detect if its a human or not, and use waypoint distance to calculate if its close to waypoint and must return home
confidence = 0
while confidence < 0.5 and distance_wp > 2:
    # capture the next image
    img = input.Capture()
    
    
    #detect objects in the image (with overlay)
    detections = net.Detect(img, overlay="box,labels,conf")
    
    # print the class id
    if len(detections) > 0:
        for detection in detections:
            counter = net.GetClassDesc(detection.ClassID)
            if counter == 'person':
                confidence = detection.Confidence
                print("Detected a person! Confidence is {:f}".format(confidence))
                jetson.utils.saveImageRGBA(os.path.join(dir_original, time_stamp + '.jpg')) # saves image the output folder


	# render the image
    output.Render(img)

    nextwaypoint = vehicle.commands.next

    if nextwaypoint == len(cmds):
        print("Going to waypoint {}".format(nextwaypoint))
        current_loc = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon]
        waypoint_last = dk.LocationGlobalRelative(cmds[len(cmds)-1].x, cmds[len(cmds)-1].y, cmds[len(cmds)-1].z)
        distance_wp = distance_to_waypoint(current_loc, [waypoint_last.lat, waypoint_last.lon])

    #current_loc = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon] #current coordinates of the drone
    #distance_wp = distance_to_waypoint(current_loc, [waypoint1.lat, waypoint1.lon]) #distance to next waypoint

if confidence > 0.5:
    print("Found person with high confidence! Breaking Loop.")
else:
    print("Didnt find anyone, reached waypoint.")
# STOP Flying --------------------------------
send_ned_velocity(0, 0, 0)  # stop the vehicle 
#sleepNrecord(2)        
time.sleep(3) #for 3 seconds
# READ CURRENT COORDINATES FROM PIXHAWK-------------------
lat = vehicle.location.global_relative_frame.lat  # get the current latitude
lon = vehicle.location.global_relative_frame.lon  # get the current longitude
coords_to_gcs = "GCS" + " " + str(lat) + " " + str(lon)
# TRANSMIT CURRENT COORDINATES TO RESCUE DR -------------- 
ser.write(coords_to_gcs.encode())

# RETURN HOME CODE ----------------------------
#set RTL_alt to something safe for two drones
vehicle.parameters['RTL_ALT'] = 0 #makes vehicle return to home at current altitude
print ("\nSet Vehicle.mode = RTL (currently: %s)" % vehicle.mode.name) 
vehicle.mode = dk.VehicleMode('RTL')
while vehicle.mode.name != 'RTL':
    print (" Waiting for changing mode")
    time.sleep(1)
print ("Mode: %s" % vehicle.mode.name)
print("Returning Home")
# vehicle.mode = dk.VehicleMode("RTL") 
print("Going Home")

# WHILE LOOP TO CONTINUE RECORDING
while vehicle.armed:
    # capture the next image
    img = input.Capture()
    # detect objects in the image (with overlay)
    detections = net.Detect(img, overlay="box,labels,conf")
    # render the image
    output.Render(img)

#time.sleep(20)

# ---------------------------------------------
#vehicle.mode = dk.VehicleMode("LAND")
vehicle.flush()