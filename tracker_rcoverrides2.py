#!/usr/bin/env python
import cv2 as cv
import pyautogui 

from picamera2 import Picamera2
from dronekit import connect, VehicleMode
from time import time, sleep

# Video resolution
dispW=720
dispH=520

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
thr = 1500
yaw = 1500
roll = 1500
pitch = 1500

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

pyautogui.moveTo(dispW // 2, dispH - 1)
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

def get_yaw():
    global yaw
    if 1490 <= yaw <= 1510:
        yaw = 1500
    if yaw > 1510:
        yaw = yaw - 10
    if yaw < 1490:
        yaw = yaw + 10
    return yaw

def get_pitch():
    global pitch
    if 1490 <= pitch <= 1510:
        pitch = 1500
    if pitch > 1510:
        pitch = yaw - 10
    if pitch < 1490:
        pitch = pitch + 10
    return pitch

def get_roll():
    global roll
    if 1490 <= roll <= 1510:
        roll = 1500
    if roll > 1510:
        roll = roll - 10
    if roll < 1490:
        roll = roll + 10
    return roll

def trackTarget(frame): 
    (success, box) = tracker.update(frame) 
    if success: 
        (x, y, w, h) = [int(v) for v in box] 
        cv.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2) 
 
        roll_error = (x + w / 2) - dispW / 2 
        pitch_error = (y + h / 2) - dispH / 2 
 
        # adjusted experimentally 
        K_roll = 1 
        K_pitch = 1 
        K_throttle = 0.7 
        K_yaw = 1   
 
        roll_command = int(get_roll() + K_roll * roll_error) 
        pitch_command = int(get_pitch() - K_pitch * pitch_error) 
        throttle_command = int(thr - K_throttle * pitch_error)  
        yaw_command = int(get_yaw() + K_yaw * roll_error)  
 
        # Limiting command values ​​to an acceptable range 
        roll_command = max(1200, min(roll_command, 1800)) 
        pitch_command = max(1200, min(pitch_command, 1800)) 
        throttle_command = max(1200, min(throttle_command, 1700))   
        yaw_command = max(1200, min(yaw_command, 1800))  
 
        rcOverrides(roll_command, pitch_command, throttle_command, yaw_command) 
 
    return success, frame

while True:
#    tStart = time()
    frame = picam2.capture_array()
    frame = cv.flip(frame, -1)

    dispW = frame.shape[1]
    dispH = frame.shape[0]
    x = dispW // 2
    y = dispH // 2

    # Crop a region of interest (ROI) from the frame
    roi = frame[(y-25):(y+25), (x-25):(x+25)] 
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
    
    cv.namedWindow("Frame", cv.WND_PROP_FULLSCREEN)
    cv.setWindowProperty("Frame", cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
    cv.imshow("Frame", frame)

    key = cv.waitKey(1) & 0xFF
    
    # Select Region on Interest (ROI) to track
#    if key == ord("c"):
    if vehicle.channels['6'] > 1800 and BB is None:
        roll = vehicle.channels['1']
        pitch = vehicle.channels['2']
        thr = vehicle.channels['3']
        yaw = vehicle.channels['4']
        BB = (x-25, y-25, 50, 50)        
        tracker.init(frame, BB)

    # Stop tracking
#    if key == ord("v"):
    if vehicle.channels['6'] < 1800:
        BB = None

    # Close video window
    if key == ord("q"):
            vehicle.close()
            break

    # FPS count
#    tEnd=time()
#    loopTime=tEnd-tStart
#    print(loopTime)
#    fps=.9*fps + .1*(1/loopTime)

# Stop tracking
cv.destroyAllWindows()
