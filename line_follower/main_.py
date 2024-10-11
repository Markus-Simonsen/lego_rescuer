#!/usr/bin/env pybricks-micropython
# Question 1: Can we assume that the tomato can will be placed directly in front of the end of the line?
# Question 2: Ramp management, do we just film one take going up and one take going down?
# Question 3: Angle of can?
"""
Example LEGO MINDSTORMS EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""
# ---------------------------------------------------------------------------- #
#                                    Import                                    #
# ---------------------------------------------------------------------------- #
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import time
from rescuer import Rescuer


# ---------------------------------------------------------------------------- #
#                                    Classes                                   #
# ---------------------------------------------------------------------------- #


def main():
    jerry = Rescuer()
    while True:
        jerry.behaviour_tree()


if __name__ == "__main__":
    main()
