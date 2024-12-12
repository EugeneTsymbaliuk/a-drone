#!/usr/bin/env python
"""
Upon running script vehicle takes off on 50 meters in alt hold. That's why you need set throttle in middle position to hover at 50 meters.
After reaching 50 meters you can fly manually in alt hold mode. 
AUX5 in position up enables autonomous flying mode. AUX5 down anables stabilize mode.  
"""

from dronekit import connect, VehicleMode
from time import sleep

# Channels1-4 values
thr = 1700
yaw = 1500
roll = 1500
pitch = 1200

# Bounding Box
BB = None

# Wait 60 seconds
print("Wait 60 seconds")
sleep(60)

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762')
#vehicle = connect('tcp:10.78.28.77:5762')
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def armAndtakeoff(alt):
    while not vehicle.armed:
        print("Arming vehicle")
        vehicle.armed = True
        sleep(1)
    print("Taking off on " + str(alt) + " meters")
    while vehicle.location.local_frame.down > -alt:
        if vehicle.mode.name != "ALT_HOLD":
            vehicle.mode = VehicleMode("ALT_HOLD")
        rcOverrides(roll, 1500, 1950, yaw)

while True:
    # Enable autonomous mode on channel 5
    if vehicle.channels['5'] > 1800:
        # Enable stabilize mode
        if vehicle.mode != "ALT_HOLD":
            vehicle.mode = VehicleMode("ALT_HOLD")
#        print("Autonomous flying")
        rcOverrides(roll, pitch, thr, yaw)

    # Enable manual mode on channel 5
    if vehicle.channels['5'] < 1800:
#        print("Manual Flying")
        rcOverrides(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])

    # Arm and takeoff on 10 meters on channel 8
    if vehicle.channels['8'] > 1800 and BB is None:
        BB = 1 
        armAndtakeoff(10)
        sleep(0.2)
    # Enable Landing mode
    if vehicle.channels['8'] < 1800 and BB is not None:
        if vehicle.armed:
            print("Landing")
            vehicle.mode = VehicleMode("LAND")
            sleep(0.1)