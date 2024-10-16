# Before launching this script type command "sudo pigpiod"

from dronekit import connect, VehicleMode
from gpiozero import AngularServo
from time import sleep

print('Connecting to FC')
vehicle = connect("/dev/ttyAMA0", baud=115200, wait_ready=True, timeout=100)
print('Connected')

drone_heading = vehicle.heading
#print(" Heading: %s" % vehicle.heading)


#pan = AngularServo(18, min_angle=-90, max_angle=90)
pan = AngularServo(18, min_pulse_width=0.0006, max_pulse_width=0.0023)
#tilt = AngularServo(17, min_pulse_width=0.0006, max_pulse_width=0.0023)
pan.angle = 0
panAngle = 0

while True:
    if vehicle.heading > drone_heading:
#        if 355 < vehicle.heading < 359:
#            drone_heading = 360 - vehicle.heading
        panAngle = panAngle + (drone_heading - vehicle.heading)
        if panAngle < -90:
            panAngle = -90
        pan.angle = panAngle  
        drone_heading = vehicle.heading
#        if drone_heading > 359:
#            drone_heading = 360 - vehicle.heading
#        panAngle = panAngle - 1
#        if panAngle < -90:
#            panAngle = -90
#        pan.angle = panAngle

    if vehicle.heading < drone_heading:
#        if 0 < vehicle.heading < 5:
#            drone_heading = 359 + vehicle.heading
        panAngle = panAngle + (drone_heading - vehicle.heading)
        if panAngle > 90:
            panAngle = 90
        pan.angle = panAngle
        drone_heading = vehicle.heading
#        if drone_heading < 1:
#            drone_heading = 359 + vehicle.heading
#        panAngle = panAngle + 1
#        if panAngle > 90:
#            panAngle = 90
#        pan.angle = panAngle
#    sleep(0.1)
#    else:
#        panAngle = vehicle.heading - 180
#    pan.angle = 0
