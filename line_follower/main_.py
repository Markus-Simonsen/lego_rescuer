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
from pybricks.tools import wait
from rescuer import Rescuer
# import main_sand


# ---------------------------------------------------------------------------- #
#                                    Classes                                   #
# ---------------------------------------------------------------------------- #
# TODO: Grip straight
# TODO: Fix decline with nose


def main():

    jerry = Rescuer()
    # jerry.check_turn_calibration(degrees_180=494)
    jerry.ev3.speaker.beep()
    jerry.calibrate_line_follower(samples=5000, stop=False)
    # while True:
    #     print("Distance: ", jerry.ultrasonic_sensor.distance(), end="\r")
    while True:
        # jerry.calibrate_line_follower(samples=5000, stop=False)
        jerry.behaviour_tree()
        print("hej")


if __name__ == "__main__":
    main()
