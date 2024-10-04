#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick


def main():

    ev3 = EV3Brick()

    # Initialize the motors.
    gripper_motor = Motor(Port.A)
    gripper_motor.run_angle(100000000, 90)
    while 1:
        print(gripper_motor.angle())
