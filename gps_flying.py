#!/usr/bin/env python

from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import time, sleep

# Wait 60 seconds
print("Wait 60 seconds")
sleep(60)

# Altitude of flying
altitude = 10

# Ground speed (m/s)
gndspeed = 10

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


if __name__ == "__main__":

    arm_and_takeoff(altitude)

    #print("Set default/target airspeed to 3")
    #vehicle.airspeed = 3

    print("Going towards first point for 60 seconds ...")
    point1 = LocationGlobalRelative(50.394958, 30.992676, altitude)
    vehicle.simple_goto(point1, groundspeed=gndspeed)

    # sleep so we can see the change in map
    sleep(60)

    #print("Going towards second point for 60 seconds (groundspeed set to 10 m/s) ...")
    #point2 = LocationGlobalRelative(50.394663, 30.996779, altitude)
    #vehicle.simple_goto(point2, groundspeed=gndspeed)

    # sleep so we can see the change in map
    #sleep(60)

    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()