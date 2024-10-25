#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick
# from PID import PID_controller


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


def main_sand():
     # Initialize the EV3 brick.
    ev3 = EV3Brick()

    # Initialize the motors.
    gripper_motor = Motor(Port.A)
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)

    # Initialize the color sensor.
    line_sensor_right = ColorSensor(Port.S1)
    touch_sensor = TouchSensor(Port.S2)
    light_sensor = LightSensor(Port.S3)
    line_sensor_left = ColorSensor(Port.S4)
    left_motor.speed(100)


# Run main
if __name__ == "__main__":
    main_sand()
