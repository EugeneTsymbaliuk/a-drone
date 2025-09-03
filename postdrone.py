#!/usr/bin/env python

import geopy.distance
from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import time, sleep

# Altitude of flying
altitude = 20

# Ground speed (m/s)
gndspeed = 15

BB = None

roll, pitch, thr, yaw = (1500, 1500, 1500, 1500)

# Create the connection to drone
#print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
#print('Connected to FC')

def changeCoords():
    global coords
    with open("/home/simba/Desktop/coords2.txt") as f:
        for i in f:
            coord = i.rstrip('\n')
            coords = coord.split(", ")
    st_p = [vehicle.location.global_frame.lat, vehicle.location.global_frame.lon]
    #st_p = [55.000000, 33.00000]
    st_p = ', '.join([str(s) for s in st_p])
    with open("/home/simba/Desktop/coords2.txt", "w") as f2:
        f2.write(st_p)

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    vehicle.mode = VehicleMode("ALT_HOLD")
    sleep(0.2)

    while not vehicle.armed:
        vehicle.armed = True
        #print(" Waiting for vehicle to initialise...")
        sleep(4)

    while vehicle.location.global_relative_frame.alt < aTargetAltitude:
        #print("TAKING OFF!")
        rcOverrides(roll, pitch, 1900, yaw)
        sleep(0.5)

def fly():
    global BB

    while True:
        if vehicle.channels['9'] > 1800:
            if vehicle.gps_0.satellites_visible < 6 and vehicle.mode == "GUIDED":
                #print("NOT ENOUGH SATS!")
                if vehicle.mode != "ALT_HOLD":
                    vehicle.mode = VehicleMode("ALT_HOLD")
                    sleep(0.2)
                rcOverrides(roll, 1900, thr, yaw)

            # Fly to appointed gps coordinate
            if vehicle.gps_0.satellites_visible > 6:
                if BB is None:
                    changeCoords()
                    arm_and_takeoff(altitude)
                    BB = 1

                # Copter should arm in GUIDED mode
                if vehicle.mode != "GUIDED":
                    vehicle.mode = VehicleMode("GUIDED")
                    sleep(0.2)
                    #print("Going towards target...")
                    point1 = LocationGlobalRelative(float(coords[0]), float(coords[1]), altitude)
                    vehicle.simple_goto(point1, groundspeed=gndspeed)

                if geopy.distance.geodesic([vehicle.location.global_frame.lat, vehicle.location.global_frame.lon], [float(coords[0]), float(coords[1])]).m < 2.0:
                    vehicle.mode = VehicleMode("LAND")
                    sleep(1)
                    # Close vehicle object before exiting script
                    #print("Close vehicle object and stoping the code")
                    vehicle.close()
                    break

        if vehicle.channels['9'] < 1800:
            BB = None
               
        sleep(0.5)
            
if __name__ == "__main__":
    fly()