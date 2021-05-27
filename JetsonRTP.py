# Use to stream a video device to the RTP

import jetson.inference
import jetson.utils

import argparse
import sys

import cv2

from datetime import datetime
import os


# Video Device
#input = jetson.utils.videoSource("v4l2:///dev/video1", [ '–input-width=1920', '–input-height=1080, -input-rtsp-latency=0']) #USB camera
input = jetson.utils.videoSource("csi://0") # MIPI CSI camera
#input = jetson.utils.videoSource("csi://0", [ '–input-width=1920', '–input-height=1080']) #MIPI camera with width and height adjusted

# Testing IR Camera CURRENTLY DOESN'T WORK.
# gst_str = ('nvarguscamerasrc !' 
#     'video/x-raw(memory:NVMM), '
#     'width=(int)320, height=(int)256, '
#     'format=(string)NV12, framerate=30/1 ! '
#     'nvvidconv ! '
#     'video/x-raw, format=(string)BGRx ! '
#     'videoconvert ! '
#     'video/x-raw, format=(string)RGB ! appsink')

# input = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

#use for if need to read from a recording
#input = jetson.utils.videoSource('/home/souyu/Desktop/TargetRecVideos/myvideo.avi')

output = jetson.utils.videoOutput("rtp://192.168.133.51:1234") # use for remote viewing, else comment out

print("Jetson Camera RTP stream started!")
# run image detection
confidence = 0
while confidence < 1:
    # capture the next image
    img = input.Capture()
	#img = input.read()
    
    #render the image
    output.Render(img)
    #cv2.imshow("cam", img)

	# update the title bar
	#output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance infojtop
	#net.PrintProfilerTimes()

	# exit on input/output EOS
    if not input.IsStreaming() or not output.IsStreaming():
        break