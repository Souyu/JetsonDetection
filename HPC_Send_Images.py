import subprocess
import jetson.inference
import jetson.utils

import cv2

from datetime import datetime
import os

# load the object detection network
net = jetson.inference.detectNet()
input = jetson.utils.videoSource()

#use for if need to read from a recording
#input = jetson.utils.videoSource('/home/souyu/Desktop/TargetRecVideos/myvideo.avi')

#output = jetson.utils.videoOutput("rtp://192.168.87.48:1234") # use for remote viewing, else comment out

#record to avi
dir_original ='/home/souyu/Desktop/TargetImages'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
output = jetson.utils.videoOutput(os.path.join(dir_original, time_stamp + '.mp4'))

# run image detection
print("Image Detection has started!")
print(time_stamp)
confidence = 0
while confidence < 0.5:
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

    if not input.IsStreaming() or not output.IsStreaming():
        break

result = subprocess.run(['rclone', 'copy', '/home/souyu/Desktop/TargetImages', 'pomona-uav-drive:jpl_images'])