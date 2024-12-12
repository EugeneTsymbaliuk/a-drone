# Before launching this script type command "sudo pigpiod"

from picamera2 import Picamera2
import time
import cv2 as cv
import numpy as np
from dronekit import connect, VehicleMode

# Channels1-4 values
thr = 1465
yaw = 1500
roll = 1500
pitch = 1200

# Bounding Box
BB = None

# Wait 60 seconds
print("Wait 60 seconds")
time.sleep(60)

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

picam2 = Picamera2()
dispW=720
dispH=520

picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=40
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

#fps=0

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def trackTarget(frame, contours):
    cv.putText(frame, "Tracking Enabled", (5,50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
    contours = sorted(contours, key=lambda x:cv.contourArea(x), reverse=True)
    cv.drawContours(frame,contours,-1,(255,0,0),3)
    contour = contours[0]
    x,y,w,h = cv.boundingRect(contour)
    cv.rectangle(frame,(x,y), (x+w,y+h), (0,0,255), 3)
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
        rcOverrides(roll, pitch, 1400, yaw)
    if pitch_error < -20 and -20 < roll_error < 20:
        print("Up")
        rcOverrides(roll, pitch, 1600, yaw)
    if roll_error > 20 and pitch_error > 20:
        print("Right and Down")
        rcOverrides(1600, pitch, 1400, 1550)
    if roll_error > 20 and pitch_error < -20:
        print("Right and Up")
        rcOverrides(1600, pitch, 1600, 1550)
    if roll_error < -20 and pitch_error < -20:
        print("Left and Up")
        rcOverrides(1400, pitch, 1600, 1450)
    if pitch_error > 20 and roll_error < -20:
        print("Left and Down")
        rcOverrides(1400, pitch, 1400, 1450)
    if -20 < roll_error < 20 and -20 < pitch_error < 20:
        print("Fly forward")
        rcOverrides(roll, pitch, thr, yaw)
    return frame, contours


while True:
#    tStart = time.time()
    frame = picam2.capture_array()
    frame = cv.flip(frame, -1)
    frameHSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
#    cv.putText(frame, str(int(fps))+' FPS', (5,30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

    # This boundaries allow to track yellow object
    lowerBound = np.array([28,100,100])
    upperBound = np.array([36,255,255])
    myMask = cv.inRange(frameHSV, lowerBound, upperBound)
    contours, junk = cv.findContours(myMask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    
    # Activate keyboard for pressing keys
    key = cv.waitKey(1) & 0xFF

    # Track object with green square    
    if len(contours) > 0 and BB is None:
        contours = sorted(contours, key=lambda x:cv.contourArea(x), reverse=True)
        cv.drawContours(frame,contours,-1,(255,0,0),3)
        contour = contours[0]
        x,y,w,h = cv.boundingRect(contour)
        cv.rectangle(frame,(x,y), (x+w,y+h), (0,255,0), 3)
    
    # Target tracked object
    if BB is not None:
        trackTarget(frame, contours)

    # Enable targeting tracked object  
    if key == ord("c"):
#    if vehicle.channels['5'] > 1800 and BB is None:
        BB = 1
        # Enable stabilize mode
        if vehicle.mode != "STABILIZE":
            vehicle.mode = VehicleMode("STABILIZE")
        
    # Disable targeting tracked object
    if key == ord("v"):
#    if vehicle.channels['5'] < 1800:
        BB = None
        rcOverrides(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])

    # Launch window from camera
    cv.imshow("Camera", frame)

    # Close window form camera
    if cv.waitKey(1)==ord('q'):
        break

    # Count time
#    tEnd=time.time()
#    loopTime=tEnd-tStart
#    print(loopTime)
#    fps=.9*fps + .1*(1/loopTime)

# Stop tracking
cv.destroyAllWindows()
