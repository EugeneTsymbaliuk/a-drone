#!/usr/bin/env python

import cv2 as cv
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

# Wait 60 seconds
print("Wait 60 seconds")
#sleep(60)

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.140:5762')
#vehicle = connect('tcp:194.247.173.68:5773')
vehicle = connect("/dev/ttyAMA0", baud=115200, wait_ready=True,  timeout=100, rate=15)
print('Connected to FC')

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
#thr = 1600
#thr_down = 1400
#thr_up = 1800
#ch3 = 0
roll = 1500
yaw = 1500
#roll_right = 1600
#roll_left = 1400
#yaw_right = 1520
#yaw_left = 1480

'''
def arm():
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        sleep(1)
    print("Arming motors")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        sleep(1)
    vehicle.mode = VehicleMode("GUIDED")

def takeOff(high):
    print("Taking off on 20 meters")
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
                                                      0, 0, 0, 0, 0, 0, 0, high)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def changAlt(alt):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_CHANGE_ALTITUDE, 
                                                      alt, MAV_FRAME_BODY_OFFSET_NED, 0, 0, 0, 0, 0)
    # send command to vehicle
    vehicle.send_mavlink(msg)
'''

def rcOverrides(roll, pitch, thr, yaw):
#    print(roll, pitch, thr, yaw)
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

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
            ch1 = roll + 100
            ch4 = yaw + 10
        elif yaw_error < -10:
            print("Yaw left")     
            ch1 = roll - 100
            ch4 = yaw - 10
#        elif pitch_error > 10:
#            print("Throttle down")
#            if thr < 1300:
#                thr = 1350
#            ch1 = roll
#            thr = thr - 3
#            ch4 = yaw
#        elif pitch_error < -10:
#            print("Throttle up")
#            if thr > 1900:
#                thr = 1850
#            ch1 = roll
#            ch3 = thr + 100
#            ch4 = yaw
        else:
            print("Fly Forward")
#            thr = start_thr + 10
            ch1, ch4 = (roll, yaw)
#        print(ch1, pitch, ch3, ch4)
        rcOverrides(ch1, pitch, thr, ch4)
    return success, frame
#arm()
#takeOff(20)
#sleep(15)
#flyBodyoffset(1000, 0, -10)


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
        cv.putText(frame, "Throttle: " + str(thr), (5,70), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        success, frame = trackTarget(frame) # Track object
        

    cv.putText(frame, "Connected", (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
    cv.imshow("Frame", frame)
    key = cv.waitKey(1) & 0xFF
    
#    chan1, chan2, chan3, chan4 = (vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])
#    rcOverrides(chan1, chan2, chan3, chan4)

    # Select Region on Interest (ROI) to track
#    if key == ord("c"):
    if vehicle.channels['5'] > 1550 and BB is None:
#        print("Object is tracking")
 
        # Set red square coordinates. For 720x520 resolution
        BB = (335, 235, 50, 50)        

        # Enable stabilize mode
        if vehicle.mode != "STABILIZE":
#            print("Stabilize mode enab12led")
            vehicle.mode = VehicleMode("STABILIZE")

        # Save throttle and pitch values
        pitch = vehicle.channels['10']
        thr = vehicle.channels['11']
#        pitch = 1300
#        thr = 1600     
        # Run tracking
        tracker.init(frame, BB)

    # Stop tracking
#    if key == ord("v"):
    if vehicle.channels['5'] < 1550:
        BB = None
#        print(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])
        rcOverrides(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])
    # Close video window
    if key == ord("q"):
            # Close vehicle object
            vehicle.close()
            break
    tEnd=time()
    loopTime=tEnd-tStart
    print(loopTime)

# Stop tracking
cv.destroyAllWindows()