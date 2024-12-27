#!/usr/bin/env python

import cv2 as cv
from picamera2 import Picamera2
from dronekit import connect, VehicleMode
from time import time, sleep

# Video resolution
dispW=720
dispH=480

# ROI size
roi_size = 150

# Center of video window
x = dispW // 2
y = dispH // 2

# FPS value for test
#fps=0

# Bounding Box
BB = None

# Channels1-4 values
#thr = 1500
yaw = 1500
roll = 1500
#pitch = 1200

# Wait 60 seconds
#print("Wait 60 seconds")
#sleep(60)

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

# Create picamera instance
picam2 = Picamera2()

# Video Settings
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=40
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Object tracker
tracker = cv.TrackerCSRT_create() # Initialize tracker with CSRT algorithm

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def merge_roi(frame, roi, x, y):
    frame[y:y+roi_size, x:x+roi_size, :] = roi
    return frame

def trackTarget(frame):
    (success, box) = tracker.update(frame)
    if success:
        (x, y, w, h) = [int(v) for v in box]
        cv.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        roll_error = (x + w/2) - dispW/2
        pitch_error = (y + h/2) - dispH/2
        if roll_error > 20 and -20 < pitch_error < 20:
            print("Right")
            rcOverrides(1600, pitch, thr, 1550)
        if roll_error < -20 and -20 < pitch_error < 20:
            print("Left")
            rcOverrides(1400, pitch, thr, 1450)
        if pitch_error > 20 and -20 < roll_error < 20:
            print("Down")
            rcOverrides(roll, pitch, thr-50, yaw)
        if pitch_error < -5 and -20 < roll_error < 20:
            print("Up")
            rcOverrides(roll, pitch, thr+50, yaw)
        if roll_error > 20 and pitch_error > 20:
            print("Right and Down")
            rcOverrides(1600, pitch, thr-50, 1550)
        if roll_error > 20 and pitch_error < -5:
            print("Right and Up")
            rcOverrides(1600, pitch, thr+50, 1550)
        if roll_error < -20 and pitch_error < -5:
            print("Left and Up")
            rcOverrides(1400, pitch, thr+50, 1450)
        if pitch_error > 20 and roll_error < -20:
            print("Left and Down")
            rcOverrides(1400, pitch, thr-50, 1450)
        if -20 < roll_error < 20 and -5 < pitch_error < 20:
            print("Fly forward")
            rcOverrides(roll, pitch, thr, yaw)
    else:
        rcOverrides(roll, pitch, thr, yaw)
    return success, frame

while True:
#    tStart = time()
    frame = picam2.capture_array()
    frame = cv.flip(frame, -1)

    # Crop a region of interest (ROI) from the frame
    roi = frame[y-25:y+25, x-25:x+25]

    # Draw rectangle in the center. For 640x480 resolution
    cv.rectangle(frame, (x-25, y+25), (x+25, y-25), (0, 255, 0), 2)

    # Resize the ROI to a specific size (e.g., 200x200)
    roi_resized = cv.resize(roi, (roi_size, roi_size))

    # Merge the resized ROI back into the frame
    frame = merge_roi(frame, roi_resized, (dispW-roi_size-20), 0)
    
    if BB is not None:
        cv.putText(frame, "Tracking Enabled", (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
#        cv.putText(frame, "Throttle: " + str(thr), (5,70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        success, frame = trackTarget(frame) # Track object
        

    cv.putText(frame, "Connected", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
#    cv.putText(frame, str(int(fps))+' FPS', (5,70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)
    cv.imshow("Frame", frame)
    key = cv.waitKey(1) & 0xFF
    
    # Select Region on Interest (ROI) to track
#    if key == ord("c"):
    if vehicle.channels['6'] > 1800 and BB is None:
#        print("Object is tracking")
 
        # Set red square coordinates. For 720x520 resolution
        BB = (x-25, y-25, 50, 50)        
        
        pitch = vehicle.channels['2']
        thr = vehicle.channels['3']
        
        # Enable stabilize mode
        #if vehicle.mode != "Stabilize":
        #    vehicle.mode = VehicleMode("Stabilize")

        # Save throttle and pitch values
#        pitch = vehicle.channels['10']
#        thr = vehicle.channels['11']
        # Run tracking
        tracker.init(frame, BB)

    # Stop tracking
#    if key == ord("v"):
    if vehicle.channels['6'] < 1800:
        BB = None
#        print(vehicle.channels['5'], vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])
        rcOverrides(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])

    # Close video window
    if key == ord("q"):
            # Close vehicle object
            vehicle.close()
            break

    # FPS count
#    tEnd=time()
#    loopTime=tEnd-tStart
#    print(loopTime)
#    fps=.9*fps + .1*(1/loopTime)

# Stop tracking
cv.destroyAllWindows()
