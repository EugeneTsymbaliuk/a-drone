#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dronekit import connect
from time import sleep

# Create the connection to drone
print('Connecting to FC')
vehicle = connect('tcp:10.78.28.77:5762')
#vehicle = connect('tcp:194.247.173.68:5773')
#vehicle = connect("/dev/ttyAMA0", baud=115200, wait_ready=True, timeout=100)
print('Connected to FC')

while True:
#    print("Roll: %s" % vehicle.channels['1'])
#    print("Pithc: %s" % vehicle.channels['2'])
#    print("Throttle: %s" % vehicle.channels['3'])
#    print("Yaw: %s" % vehicle.channels['4'])
    # Print pitch attitude. Same can reflect fo roll and yaw
    #pitch = vehicle.attitude.pitch * 1000 + 1500
    #print(pitch)
    print(" Local Location: %s" % vehicle.location.local_frame)
    sleep(0.1)

#vehicle = connect('tcp:194.247.173.68:5772')
# Get all vehicle attributes (state)
#print("\nGet Flight Mode:")
#print(" Mode: %s" % vehicle.mode.name)    # settable
#print(" Attitude: %s" % vehicle.attitude)
#print(" Global Location: %s" % vehicle.location.global_frame)
#print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
#print(" Local Location: %s" % vehicle.location.local_frame)
#print(" Heading: %s" % vehicle.heading)
#Close vehicle object before exiting script

print("\nClose vehicle object")
vehicle.close()
