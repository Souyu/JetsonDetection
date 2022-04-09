# coding: utf-8


import cv2
import time
import dronekit as dk
from pymavlink import mavutil
import os

from datetime import datetime

#Directory
full_path = os.path.realpath(__file__)
dir = os.path.dirname(full_path)

#Mission txt file
mission_name = 'Spadra_Farm_Search_WPS.txt'

# Before initializing, wait for a press of a button
#print("Now waiting for Button Press...")
#btn="off"
#while btn !="on": #if incoming bytes are waiting to be read from the serial input buffer
#    btn=device.read_data()
#    if btn is not None:
#        btn = btn.data.decode()

# starting up dependencies
from Defs.PixHawk import *
from Defs.Xbee import *


import jetson.inference
import jetson.utils


input = jetson.utils.videoSource()
#input = jetson.utils.videoSource('/dev/video1') #USB camera
#output = jetson.utils.videoOutput("rtp://192.168.79.51:1234") # use for remote viewing, else comment out

#record to mp4 with timestamp
dir_original ='/home/souyu/Desktop/TargetRecRecording'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
output = jetson.utils.videoOutput(os.path.join(dir_original, time_stamp + '.mp4'))

# load the object detection network
net = jetson.inference.detectNet()

# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)# PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB
#parser= argparse.ArgumentParser(description= 'commands')
#parser.add_argument('--connect')
#args=parser.parse_args()
#connection_string= args.connect

print("Connection to the vehicle on %s" %connection_string)

#vehicle= dk.connect(connection_string, wait_ready=True)

#recieving the waypoints from GCS
recieve_GPS_coord_xbee() 

# uploading mission waypoint from text
missionwp = readmission(mission_name)

print ("\nUpload mission from a file: %s" % mission_name)
# Clear existing mission from vehicle
print ("Clear mission")
cmds = vehicle.commands
cmds.clear()
# Add new mission to vehicle
for command in missionwp:
    cmds.add(command)
print ("Upload mission")
cmds.upload()
print(mission_name)

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
    #write GPS to text
    gpsToText(dir,time_stamp)

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
                saveimg = jetson.utils.cudaToNumpy(img) #Saves only the timestamp that was created in beginning. May need to update later
                cv2.imwrite(os.path.join(dir_original, time_stamp + '.jpg'),saveimg) # saves image the output folder

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
device.send_data_broadcast(coords_to_gcs)

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
    #GPS to Text
    gpsToText()
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