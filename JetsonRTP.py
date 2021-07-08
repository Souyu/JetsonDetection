# Use to stream a video device to the RTP

import jetson.inference
import jetson.utils

import argparse
import sys

from datetime import datetime
import os


# Video Device
#input = jetson.utils.videoSource("v4l2:///dev/video0") #USB camera
#input = jetson.utils.videoSource("csi://0") # MIPI camera
input = jetson.utils.videoSource("csi://0", [ '–input-width=1920', '–input-height=1080']) #MIPI camera with width and height adjusted

#use for if need to read from a recording
#input = jetson.utils.videoSource('/home/souyu/Desktop/TargetRecVideos/myvideo.avi')

output = jetson.utils.videoOutput("rtp://192.168.228.51:1234") # use for remote viewing, else comment out

print("Jetson Camera RTP stream started!")
# run image detection
confidence = 0
while confidence < 1:
    # capture the next image
    img = input.Capture()
    
	# render the image
    output.Render(img)

	# update the title bar
	#output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance infojtop
	#net.PrintProfilerTimes()

	# exit on input/output EOS
    if not input.IsStreaming() or not output.IsStreaming():
        break