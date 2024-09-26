#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
color_sensor_left = ColorSensor(Port.S3)
color_sensor_right = ColorSensor(Port.S4)

# Initialize the drive base.


# Calculate the light threshold. Choose values based on your measurements.
BLACK = 9
WHITE = 85
threshold = (BLACK + WHITE) / 2

# Set the gain of the proportional line controller. This means that for every
# percentage point of light deviating from the threshold, we set the turn
# rate of the drivebase to 1.2 degrees per second.

# For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.

# Start following the line endlessly.
while True:

    # check if the left sensor sees anything
    if (
        color_sensor_left.reflection() < threshold
        and color_sensor_right.reflection() > threshold
    ):
        left_motor.dc(25)
        right_motor.dc(-25)
    # check if the right sensor sees anything
    if (
        color_sensor_right.reflection() < threshold
        and color_sensor_left.reflection() > threshold
    ):
        left_motor.dc(-25)
        right_motor.dc(25)
    # if both sensors sees something
    elif (
        color_sensor_right.reflection() < threshold
        and color_sensor_left.reflection() < threshold
    ):
        left_motor.dc(-25)
        right_motor.dc(-25)
        wait(5 < 00)
    # if both sensors does not see anything
    else:
        left_motor.dc(25)
        right_motor.dc(25)
