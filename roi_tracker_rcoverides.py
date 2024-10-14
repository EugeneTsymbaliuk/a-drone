#!/usr/bin/env python
import math
import cv2 as cv
from picamera2 import Picamera2
#from pymavlink import mavutil
#from dronekit import connect, VehicleMode
import threading
from multiprocessing import Process
import time

# Video resolution
dispW=720
dispH=520
roi_size = 200
fps = 0

# Center of video window
x = dispW // 2
y = dispH // 2

# Wait 60 seconds
#print("Wait 60 seconds")
#time.sleep(60)

# Create the connection to drone
#print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.140:5762')
#vehicle = connect('tcp:194.247.173.68:5773')
#vehicle = connect("/dev/ttyAMA0", baud=115200, wait_ready=True, timeout=100)
#print('Connected to FC')
#boot_time = time.time()

# Create picamera instance
picam2 = Picamera2()

# Video Settings
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Object tracker
tracker = cv.TrackerCSRT_create() # Initialize tracker with CSRT algorithm

BB = None # Bounding Box
'''
thr = 1600
thr_down = 1400
thr_up = 1800
roll = 1500
yaw = 1500
roll_right = 1600
roll_left = 1400
yaw_right = 1520
yaw_left = 1480

def arm():
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

def takeOff(high):
    print("Taking off on 20 meters")
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                                      0, 0, 0, 0, 0, 0, 0, high)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}
'''
def merge_roi(frame, roi, x, y):
    frame[y:y+roi_size, x:x+roi_size, :] = roi
    return frame

def moveRedsquare(frame, x, y, w, h):
    cv.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
    tracker.init(frame, (x, y, w, h))

def trackTarget(frame):
    (success, box) = tracker.update(frame)
    if success:
        (x, y, w, h) = [int(v) for v in box]
        cv.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        yaw_error = (x + w/2) - dispW/2
        pitch_error = (y + h/2) - dispH/2
#        print(yaw_error)
#        if pitch_error > 10:
#            print("Down")
#            rcOverrides(roll, pitch, thr_down, yaw)
#        elif pitch_error < -10:
#            print("Up")
#            rcOverrides(roll, pitch, thr_up, yaw)
#        elif yaw_error > 10:
#            print("Turn Right")
#            rcOverrides(roll_right, pitch, thr, yaw_right)
#        elif yaw_error < -10:
#            print("Turn Left")
#            rcOverrides(roll_left, pitch, thr, yaw_left)
#        else:
#            print("Fly Forward")
#            rcOverrides(roll, pitch, thr, yaw)
    return success, frame

#arm()
#takeOff(20)
#time.sleep(15)
#flyBodyoffset(1000, 0, -10)


while True:
    tStart = time.time()
    frame = picam2.capture_array()
    frame = cv.flip(frame, -1)

    # Turn stream into grey color
#    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Crop a region of interest (ROI) from the frame
    # ROI for object tracking
    roi = frame[160:360, 260:460]
    # ROI for displaying in the corner
    roi2 = frame[235:285, 335:385]
    # For 640x480 resolution
    cv.rectangle(frame, (x-25, y+25), (x+25, y-25), (0, 255, 0), 2)

    # Resize the ROI to a specific size (e.g., 150x150)
    roi_resized = cv.resize(roi2, (roi_size, roi_size))

    # Merge the resized ROI back into the frame
    frame = merge_roi(frame, roi_resized, (dispW - roi_size - 20), 0)
    
    if BB is not None:
        cv.putText(frame, "Tracking Enabled", (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        cv.putText(frame, str(int(fps))+ ' FPS', (5,70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        success, roi_frame = trackTarget(roi)

    cv.putText(frame, "Connected", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv.putText(frame, str(int(fps))+ ' FPS', (5,90), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
    cv.imshow("Frame", frame)
    key = cv.waitKey(1) & 0xFF

    # Select Region on Interest (ROI) to track
    if key == ord("c"):
#    if vehicle.channels['8'] > 1900 and vehicle.mode == "ACRO":

        print("Object is tracking")
        # For ROI
        BB =  (75, 75, 50, 50)    
#        BB = (190, 190, 50, 50)
        # Fly forward
#        vehicle.mode = VehicleMode("STABILIZE")
        #pitch = vehicle.channels['2']
#        pitch = 1300
#        rcOverrides(roll, pitch, thr, yaw)
        
        # For 1080x720 resolution
#        BB = (440, 256, 200, 206)

        # Run tracking
        tracker.init(roi, BB)
    # Stop tracking
    if key == ord("v"):
#    elif vehicle.channels['7'] > 1900 and vehicle.mode == "STABILIZE":
#        # Switch vehicle in STABILIZE Mode
#        vehicle.mode = VehicleMode("ACRO")
        BB = None
    # Close video window
    elif key == ord("q"):
            # Close vehicle object
#            vehicle.close()
            break

    tEnd=time.time()
    loopTime=tEnd-tStart
    print(loopTime)
    fps=.9*fps + .1*(1/loopTime)

# Stop tracking
cv.destroyAllWindows()
print("End script!")
