#!/usr/bin/env python

from dronekit import connect, VehicleMode
from time import time, sleep

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.145:5762', rate=40)
vehicle = connect("/dev/ttyAMA0", baud=57600, wait_ready=True,  timeout=100, rate=40)
print('Connected to FC')

if __name__ == "__main__":
    while True:
        # Get all vehicle attributes (state)

        #print("\nGet all vehicle attribute values:")
        #print(" Autopilot Firmware version: %s" % vehicle.version)
        #print("   Major version number: %s" % vehicle.version.major)
        #print("   Minor version number: %s" % vehicle.version.minor)
        #print("   Patch version number: %s" % vehicle.version.patch)
        #print("   Release type: %s" % vehicle.version.release_type())
        #print("   Release version: %s" % vehicle.version.release_version())
        #print("   Stable release?: %s" % vehicle.version.is_stable())
        #print(" Autopilot capabilities")
        #print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
        #print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
        #print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
        #print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
        #print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
        #print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
        #print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
        #print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
        #print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
        #print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
        #print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
        #print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
        #print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
        #print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
        #print(" Global Location: %s" % vehicle.location.global_frame)
        #print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
        #print(" Local Location: %s" % vehicle.location.local_frame)
        print(" Attitude: %s" % vehicle.attitude)
        #print(" Velocity: %s" % vehicle.velocity)
        print(" GPS: %s" % vehicle.gps_0)
        #print(" Gimbal status: %s" % vehicle.gimbal)
        #print(" Battery: %s" % vehicle.battery)
        #print(" EKF OK?: %s" % vehicle.ekf_ok)
        #print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
        #print(" Rangefinder: %s" % vehicle.rangefinder)
        #print(" Rangefinder distance: %s" % vehicle.rangefinder.distance)
        #print(" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
        print(" Heading: %s" % vehicle.heading)
        #print(" Is Armable?: %s" % vehicle.is_armable)
        #print(" System status: %s" % vehicle.system_status.state)
        #print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
        #print(" Airspeed: %s" % vehicle.airspeed)    # settable
        #print(" Mode: %s" % vehicle.mode.name)    # settable
        #print(" Armed: %s" % vehicle.armed)    # settable
        sleep(1)
        
#Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()