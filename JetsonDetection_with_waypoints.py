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

from Defs import *

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

# Before initializing, wait for a press of a button
print("Now waiting for Button Press...")
btn="off"
while btn !="on": #if incoming bytes are waiting to be read from the serial input buffer
    btn=serialread()

print("Button has been pressed!  Intializing drone...")

# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)# PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB
#parser= argparse.ArgumentParser(description= 'commands')
#parser.add_argument('--connect')
#args=parser.parse_args()
#connection_string= args.connect

print("Connection to the vehicle on %s" %connection_string)

#vehicle= dk.connect(connection_string, wait_ready=True)

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
                #jetson.utils.saveImageRGBA(os.path.join(dir_original, time_stamp + '.jpg')) # saves image the output folder


	# render the image
    output.Render(img)

    nextwaypoint = vehicle.commands.next
    print("Going to waypoint {}".format(nextwaypoint))
    if nextwaypoint == len(cmds):
        current_loc = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon]
        waypoint_last = dk.LocationGlobalRelative(cmds[len(cmds)-1].x, cmds[len(cmds)-1].y, cmds[len(cmds)-1].z)
        distance_wp = distance_to_waypoint(current_loc, [waypoint_last.lat, waypoint_last.lon])

    #current_loc = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon] #current coordinates of the drone
    #distance_wp = distance_to_waypoint(current_loc, [waypoint1.lat, waypoint1.lon]) #distance to next waypoint

if confidence >= 0.5:
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
#print ("\nSet Vehicle.mode = RTL (currently: %s)" % vehicle.mode.name) 
#vehicle.mode = dk.VehicleMode('RTL')
#while vehicle.mode.name != 'RTL':
#    print (" Waiting for changing mode")
#    time.sleep(1)
#print ("Mode: %s" % vehicle.mode.name)
print("Returning Home")
vehicle.mode = dk.VehicleMode("RTL") 
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