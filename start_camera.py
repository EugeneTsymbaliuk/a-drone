#!/usr/bin/env python

import cv2 as cv
from picamera2 import Picamera2
from time import time, sleep

# Video resolution
dispW=720
dispH=520

# ROI size
roi_size = 150

# Center of video window
x = dispW // 2
y = dispH // 2

# Create picamera instance
picam2 = Picamera2()

# Video Settings
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=22
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Object tracker
tracker = cv.TrackerCSRT_create() # Initialize tracker with CSRT algorithm

BB = None # Bounding Box

def merge_roi(frame, roi, x, y):
    frame[y:y+roi_size, x:x+roi_size, :] = roi
    return frame

def trackTarget(frame):
#    global thr
    (success, box) = tracker.update(frame)
    if success:
        (x, y, w, h) = [int(v) for v in box]
        cv.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        yaw_error = (x + w/2) - dispW/2
        pitch_error = (y + h/2) - dispH/2
        #print(yaw_error)
        if yaw_error > 10:
            print("Yaw right")     
        elif yaw_error < -10:
            print("Yaw left")     
        elif pitch_error > 10:
            print("Throttle down")
        elif pitch_error < -10:
            print("Throttle up")
        else:
            print("Fly Forward")
    return success, frame

while True:
    tStart = time()
    frame = picam2.capture_array()
    frame = cv.flip(frame, -1)

    # Crop a region of interest (ROI) from the frame
    roi = frame[235:285, 335:385]

    # Draw rectangle in the center. For 640x480 resolution
    cv.rectangle(frame, (x-25, y+25), (x+25, y-25), (0, 255, 0), 2)

    # Resize the ROI to a specific size (e.g., 200x200)
    roi_resized = cv.resize(roi, (roi_size, roi_size))

    # Merge the resized ROI back into the frame
    frame = merge_roi(frame, roi_resized, (dispW-roi_size-20), 0)
    
    if BB is not None:
        cv.putText(frame, "Tracking Enabled", (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        success, frame = trackTarget(frame) # Track object
        

    cv.putText(frame, "Connected", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv.imshow("Frame", frame)
    key = cv.waitKey(1) & 0xFF
    
    # Select Region on Interest (ROI) to track
    if key == ord("c"):
        # Set red square coordinates. For 720x520 resolution
        BB = (335, 235, 50, 50)        

        # Run tracking
        tracker.init(frame, BB)

    # Stop tracking
    if key == ord("v"):
        BB = None
        
    # Close video window
    if key == ord("q"):
        break
    tEnd=time()
    loopTime=tEnd-tStart
    print(loopTime)

# Stop tracking
cv.destroyAllWindows()
