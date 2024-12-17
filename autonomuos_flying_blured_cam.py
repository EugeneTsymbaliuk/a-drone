#!/usr/bin/env python
"""
Upon running script vehicle takes off on 50 meters in alt hold. That's why you need set throttle in middle position to hover at 50 meters.
After reaching 50 meters you can fly manually in alt hold mode. 
AUX5 in position up enables autonomous flying mode. AUX5 down anables stabilize mode.  
"""

import cv2 as cv
from picamera2 import Picamera2
from dronekit import connect, VehicleMode
from time import time, sleep
from threading import Thread

# Video resolution
dispW=720
dispH=520

# Center of video window
x = dispW // 2
y = dispH // 2

# FPS value for test
fps=0

chan5 = None

# Channels1-4 values
thr = 1700
yaw = 1500
roll = 1500
pitch = 1200

# Wait 60 seconds
print("Wait 60 seconds")
#sleep(60)

# Create the connection to drone
print('Connecting to FC')
vehicle = connect('tcp:192.168.1.145:5762', rate=40)
#vehicle = connect('tcp:10.78.28.77:5762')
#vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

# Create picamera instance
picam2 = Picamera2()

# Video Settings
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def armAndtakeoff(alt):
    while not vehicle.armed:
        vehicle.armed = True
#        sleep(1)

while True:
#    tStart = time()
    frame = picam2.capture_array()
    frame = cv.flip(frame, -1)

#    key = cv.waitKey(1) & 0xFF
    
#    if key == ord("c"):
#        chan5 = 1
#    if key == ord("v"):
#        chan5 = None

#    if chan5 is not None:
    if vehicle.channels['5'] > 1800:
        frame[0:int(dispH/2), int(dispW/1.5):] = [60,100,100]
        frame[:int(dispH/2), :int(dispW/3)] = [60,100,100]
        frame[int(dispH/2):, :int(dispW/2)] = [60,100,100]
        frame[int(dispH/2):, int(dispW/2):] = [60,100,100]
        cv.putText(frame, "Autonomous", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
#        cv.putText(frame, str(int(fps))+' FPS', (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
        cv.imshow("Frame", frame)  

        if vehicle.armed == False:
            t1 = Thread(target=armAndtakeoff, args=(10,))
            t1.start()
        # Enable althold mode
        if vehicle.mode != "ALT_HOLD":
            vehicle.mode = VehicleMode("ALT_HOLD")
        rcOverrides(roll, pitch, thr, yaw)

    # Enable manual mode on channel 5
#    if chan5 is None:
    if vehicle.channels['5'] < 1800:
        cv.putText(frame, "Manual", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
#        cv.putText(frame, str(int(fps))+' FPS', (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
        cv.imshow("Frame", frame)
        rcOverrides(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])

    # Close video window
#    if key == ord("q"):
        # Close vehicle object
#        vehicle.close()
#        break

    # FPS count
#    tEnd=time()
#    loopTime=tEnd-tStart
#    print(loopTime)
#    fps=.9*fps + .1*(1/loopTime)

# Stop tracking
cv.destroyAllWindows()