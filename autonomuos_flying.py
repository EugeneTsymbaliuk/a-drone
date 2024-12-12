#!/usr/bin/env python
"""
Upon running script vehicle takes off on 50 meters in alt hold. That's why you need set throttle in middle position to hover at 50 meters.
After reaching 50 meters you can fly manually in alt hold mode. 
AUX5 in position up enables autonomous flying mode. AUX5 down anables stabilize mode.  
"""

from dronekit import connect, VehicleMode
from time import sleep

# Wait 60 seconds
print("Wait 60 seconds")
#sleep(60)

#sys_state = None

# Create the connection to drone
print('Connecting to FC')
vehicle = connect('tcp:192.168.1.145:5762')
#vehicle = connect('tcp:10.78.28.77:5762')
#vehicle = connect("/dev/ttyAMA0", baud=115200, wait_ready=True,  timeout=100, rate=15)
print('Connected to FC')

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
'''
def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def armAndtakeoff(alt):
    while not vehicle.armed:
#        if vehicle.channels['6'] > 1800:
        print("Arming vehicle")
        vehicle.armed = True
        sleep(1)
#    while vehicle.channels['6'] > 1800:
#        arm()
    print("Taking off on " + str(alt) + " meters")
    while vehicle.location.local_frame.down > -alt:
        if vehicle.mode.name != "ALT_HOLD":
            vehicle.mode = VehicleMode("ALT_HOLD")
        rcOverrides(1500, 1500, 1950, 1500)

#arm()
#armAndtakeoff(10)

while True:
    # Enable autonomous mode
    if vehicle.channels['5'] > 1800:
        print("Autonomous flying")
        rcOverrides(1500, 1800, 1600, 1500)

    # Enable manual mode
    if vehicle.channels['5'] < 1800:
        if vehicle.armed:
            print("Manual Flying")
            rcOverrides(vehicle.channels['9'], vehicle.channels['10'], vehicle.channels['11'], vehicle.channels['12'])

    if vehicle.channels['8'] > 1800:
        if not vehicle.armed:
#        print(vehicle.channels['6'])
            armAndtakeoff(10)
    
    if vehicle.channels['8'] < 1800:
#        print(vehicle.channels['6'])
        if vehicle.armed:
            if vehicle.mode.name != "LAND":
                print("Landing")
                vehicle.mode = VehicleMode("LAND")
        elif vehicle.mode.name == "LAND":
            vehicle.mode = VehicleMode("STABILIZE")
        else:
            print("Vehicle landed and is disarmed")
            sleep(0.1)