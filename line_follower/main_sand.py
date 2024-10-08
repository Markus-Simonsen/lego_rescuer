#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick




def main():

    ev3 = EV3Brick()

    # Initialize the motors.
    gripper_motor = Motor(Port.A)
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    touch_sensor = TouchSensor(Port.S2)
    #gripper_motor.run_angle(100, 90)

    motor_speed = -100000

    tomato_gripped = False

    print("Press the touch sensor to close the gripper.")
    while True:
        if touch_sensor.pressed() and not tomato_gripped:
            left_motor.run(0)
            right_motor.run(0)
            print("Touch sensor pressed.")
            gripper_motor.run_angle(100, -120)
            print("Gripper closed.")
            motor_speed = 0
            tomato_gripped = True
        # Run both motors at 50
        left_motor.run(motor_speed)
        right_motor.run(motor_speed)

# Run main
if __name__ == "__main__":
    main()
