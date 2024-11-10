#!/usr/bin/env pybricks-micropython
# Question 1: Can we assume that the tomato can will be placed directly in front of the end of the line?
# Question 2: Ramp management, do we just film one take going up and one take going down?
# Question 3: Angle of can?
import os
from os import path
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
from rescuer_sci import Rescuer
# import main_sand


# ---------------------------------------------------------------------------- #
#                                    Classes                                   #
# ---------------------------------------------------------------------------- #
# TODO: Grip straight
# TODO: Fix decline with nose
# --------------------------- Initialize the logger -------------------------- #
# Create directory if it doesn't exist
def ask_to_continue():
    obs_continue = input("Make new observation?")
    if obs_continue == "n" or obs_continue == "N":
        obs_continue = 0
    else:
        obs_continue = input("Which observation should be continued?")

    assert type(obs_continue) == int, "Observation number must be an integer"

    return obs_continue


filename = "scimet_log.csv"
start_time = time.time()
log_i = 0


def main():

    jerry = Rescuer()
    # jerry.check_turn_calibration(degrees_180=494)

    obs_num = ask_to_continue()
    jerry.log_init(obs_num)

    while True:
        # jerry.calibrate_line_follower(samples=5000, stop=False)
        jerry.can_scan(120)
        print("hej")


if __name__ == "__main__":
    main()
