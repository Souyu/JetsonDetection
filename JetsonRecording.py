import jetson.inference
import jetson.utils

import argparse
import sys
import cv2

from datetime import datetime
import os

# load the object detection network
net = jetson.inference.detectNet()
input = jetson.utils.videoSource()

#use for if need to read from a recording
#input = jetson.utils.videoSource('/home/souyu/Desktop/TargetRecVideos/myvideo.avi')

#output = jetson.utils.videoOutput("rtp://192.168.79.51:1234") # use for remote viewing, else comment out

#record to avi
dir_original ='/home/souyu/Desktop/TargetRecRecording'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
output = jetson.utils.videoOutput(os.path.join(dir_original, time_stamp + '.avi'))

# run image detection
print("Image Detection has started!")
print(time_stamp)
confidence = 0
while confidence < 1:
    # capture the next image
    img = input.Capture()
    
    #detect objects in the image (with overlay)
    detections = net.Detect(img, overlay="box,labels,conf")
    
	# print the detections
    
    # print the class id
    if len(detections) > 0:
        for detection in detections:
            counter = net.GetClassDesc(detection.ClassID)
            if counter == 'person':
                confidence = detection.Confidence
                print("Detected a person! Confidence is {:f}".format(confidence))
                #jetson.utils.cudaDeviceSynchronize() # Allows to take both video and photo at same time. Comment out if this doesn't work
                saveimg = jetson.utils.cudaToNumpy(img) #Saves only the timestamp that was created in beginning. May need to update later
                cv2.imwrite(os.path.join(dir_original, time_stamp + '.jpg'),saveimg) # saves image the output folder

	# render the image
    output.Render(img)

	# update the title bar
	#output.SetStatus("{:s} | Network {:.0f} FPS".format(opt.network, net.GetNetworkFPS()))

	# print out performance infojtop
	#net.PrintProfilerTimes()

	# exit on input/output EOS
    if not input.IsStreaming() or not output.IsStreaming():
        break