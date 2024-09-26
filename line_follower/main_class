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

import rescuer

robot = rescuer.Rescuer()


while True:
    robot.manual_control()

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
