#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick
from PID import PID_controller


def turn_180(left_motor, right_motor):
    # Turn 180 degrees
    angles = 0

    # Turn speed
    speed = 100

    # Total angle
    degrees = 545

    # Run the motors
    while angles < degrees:
        left_motor.run(speed)
        right_motor.run(-speed)
        angles = left_motor.angle()
        print(angles)

    # Stop the motors
    left_motor.run(0)
    right_motor.run(0)


def main():
    PID = PID_controller(30, 0, 2, 90, 150, LightSensor(
        Port.S4), LightSensor(Port.S1), , Motor(Port.B), Motor(Port.C))


# Run main
if __name__ == "__main__":
    main()
