# coding: utf-8

import serial
import cv2
import numpy as np
import time
import dronekit as dk
from pymavlink import mavutil
import os

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


# process frames until the user exits

#Setting up GPIO
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(18, GPIO.OUT)
#p = GPIO.PWM(18, 50)
#p.start(2.5)
#time.sleep(1)
...

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
        btn=ser.read(5).decode()

# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)# PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

waypoint1 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, 5)  

arm_and_takeoff(10)
#time.sleep(5)
vehicle.airspeed = 1 # set drone speed to be used with simple_goto
vehicle.simple_goto(waypoint1)#trying to reach 1st waypoint
#time.sleep(40)															
#----------------------------------------------


# run image detection with confidence of 30% to detect if its a human or not
confidence = 0
while confidence < 0.5:
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


	# render the image
    output.Render(img)

#    if not input.IsStreaming() or not output.IsStreaming():
#        break

print("Found person with high confidence! Breaking Loop.")
# STOP Flying --------------------------------
send_ned_velocity(0, 0, 0)  # stop the vehicle 
#sleepNrecord(2)        
time.sleep(3) #for 3 seconds
# READ CURRENT COORDINATES FROM PIXHAWK-------------------
lat = vehicle.location.global_relative_frame.lat  # get the current latitude
lon = vehicle.location.global_relative_frame.lon  # get the current longitude
coords = str(lat) + " " + str(lon)
# TRANSMIT CURRENT COORDINATES TO RESCUE DR -------------- 
ser.write(coords.encode())

# RETURN HOME CODE ----------------------------
vehicle.mode    = dk.VehicleMode("RTL")

# WHILE LOOP TO CONTINUE RECORDING
while vehicle.armed():
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