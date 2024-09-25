#!/usr/bin/env python
"""
Vehicle REB avoidance script. 
Using channel 8 to proceed flying by drone while radio signal lost. Reb avoidance activated by channel 8 in position more than 1550. 
Upon signal restoration and channel 8 in position more than 1550 autonomous flying continues. Switch channel 8 to position less than 1550 to manual flying. 
If you forget to activate REB avoidance mode by channel 8 and signal lost, drone will fall down. 
"""

from dronekit import connect, VehicleMode
from time import sleep

# Wait 60 seconds
print("Wait 60 seconds")
#sleep(60)

sys_state = None

# Create the connection to drone
print('Connecting to FC')
#vehicle = connect('tcp:192.168.1.140:5762')
#vehicle = connect('tcp:194.247.173.68:5773')
vehicle = connect("/dev/ttyAMA0", baud=115200, wait_ready=True,  timeout=100, rate=15)
print('Connected to FC')

def rcOverrides(roll, pitch, thr, yaw):
    vehicle.channels.overrides = {'1': roll, '2': pitch, '3': thr, '4': yaw}

while True:
    # Enable autonomous mode
    if vehicle.channels['8'] != "NoneType":
        if vehicle.channels['8'] > 1550 and vehicle.system_status.state == "CRITICAL":
            sys_state = vehicle.system_status.state
            pitch = 1200
            throttle = 1900
            vehicle.mode = VehicleMode("STABILIZE")
            roll = 1500
            yaw = 1500
#           sleep(0.1)
        elif vehicle.channels['8'] < 1550:
            print("Manual Flying")
            sys_state = None
            sleep(0.1)    
    else:
        print("Channel 8 is NoneType!")

    if sys_state is not None:
#        vehicle.mode = VehicleMode("STABILIZE")
#        pitch = vehicle.channels['2']
#        throttle = vehicle.channels['3'] + 100
#        roll = 1500
#        yaw = 1500
        # Overrides channels 1-4
        print("Autonomous flying")
        sleep(0.04)
        print(roll, pitch, throttle, yaw)
        rcOverrides(roll, pitch, throttle, yaw)

    # Enable manual mode
#    if vehicle.channels['8'] < 1550:
#        print("Manual Flying")
#        sys_state = None
#        sleep(0.1)