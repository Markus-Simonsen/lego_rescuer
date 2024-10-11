#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import Motor, TouchSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.hubs import EV3Brick




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

    ev3 = EV3Brick()

    # Initialize the motors.
    gripper_motor = Motor(Port.A)
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    touch_sensor = TouchSensor(Port.S2)
    #gripper_motor.run_angle(100, 90)

   
    left_motor.run(100)
    right_motor.run(100)
    wait(1000)
    turn_180(left_motor, right_motor)
    motor_speed = -500
    

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
