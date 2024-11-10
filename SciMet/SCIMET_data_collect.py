#!/usr/bin/env pybricks-micropython
# ---------------------------------------------------------------------------- #
#                                    IMPORT                                    #
# ---------------------------------------------------------------------------- #
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from folder_management import create_folder, save_data, ask_to_continue
import os
import sys


# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #


def main():
    # ----------------------------- Initialize robot ----------------------------- #
    # Initialize the EV3 Brick
    ev3 = EV3Brick()

    # Initialize the motors and sensors
    right_motor = Motor(Port.C)
    left_motor = Motor(Port.B)
    dist_sensor = UltrasonicSensor(Port.S2)
    touch_sensor = TouchSensor(Port.S3)

    wheel_distance = 165
    wheel_diameter = 56

    # Initialize the drive base
    robot = DriveBase(left_motor, right_motor, wheel_diameter, wheel_distance)
    robot.reset()  # Reset angle to 0

    # Search settings
    n_samples = 30
    sample_number = 0
    search_angle = 40

    # ----------------------------- Folder management ---------------------------- #
    # If old_observation is a number it will continue from that observation
    old_observation = ask_to_continue()
    # Create a new folder
    foldername = create_folder(old_observation)

    # ----------------------------- Start the run ----------------------------- #
    print("[INFO] Press the button to start the run", file=sys.stderr)

    # Loop through samples, scan the can in front of the robot, store the data in a separate csv file and store in unique folder
    while sample_number < n_samples:
        if touch_sensor.pressed():
            # ----- Sweap the area and sample -----
            initial_angle = robot.angle()

            robot.turn(-search_angle)
            wait(200)

            # Rotate the robot at speed 15
            robot.drive(0, 15)

            # Sample data untill the robot has rotated search_angle degrees
            measurements = []
            while robot.angle() < initial_angle + search_angle:
                measurements.append((robot.angle(), dist_sensor.distance()))
            robot.stop()
            # print("data= ", measurements)

            # Increament sample number
            sample_number = len(os.listdir(foldername)) + 1

            # ----- Save the data to a csv file -----
            save_data(measurements, foldername)
            robot.turn(-search_angle)
            print("---------------------- Run", sample_number,
                  "completed. -----------------------\n")
            print("[INFO] Press the button to start next run ")

        else:
            wait(100)

    print("[INFO] All runs completed", file=sys.stderr)


if __name__ == "__main__":
    main()
