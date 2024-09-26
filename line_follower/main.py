#!/usr/bin/env pybricks-micropython

"""
Example LEGO MINDSTORMS EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick

# # Initialize the EV3 brick.
ev3 = EV3Brick()

# # Initialize the motors.
right_motor = Motor(Port.C)
left_motor = Motor(Port.B)

# Initialize the color sensor.
line_sensor_left = ColorSensor(Port.S4)
line_sensor_right = ColorSensor(Port.S1)

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 20
WHITE = 85
threshold = (BLACK + WHITE) / 2

# # Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100

# # Set the gain of the proportional line controller. This means that for every
# # percentage point of light deviating from the threshold, we set the turn
# # rate of the drivebase to 1.2 degrees per second.

# # For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
left_speed = 100
right_speed = 100

# ev3.speaker.play_file("/music.wav")

# # Start following the line endlessly.
while True:
    # if the left sensor sees the line, turn left; under the threshold = black
    if line_sensor_left.reflection() < threshold:
        left_speed = -25
        right_speed = 25
        print(1)
    # if the right sensor sees the line, turn right
    elif line_sensor_right.reflection() < threshold:
        left_speed = 25
        right_speed = -25
        print(2)
    elif (
        line_sensor_left.reflection() < threshold
        and line_sensor_right.reflection() < threshold
    ):
        left_speed = -25
        right_speed = -25
        wait(1000)
        print(4)
    else:
        left_speed = 30
        right_speed = 30
        print(3)

    ev3.screen.draw_text(50, 500, line_sensor_left.reflection())
    ev3.screen.draw_text(2, 100, line_sensor_right.reflection())
    print(
        "left: "
        + str(line_sensor_left.reflection())
        + "right: "
        + str(line_sensor_right.reflection())
    )

    # Set the motor speeds.
    left_motor.dc(left_speed)
    right_motor.dc(right_speed)

    # You can wait for a short time or do other things in this loop.
    wait(50)
