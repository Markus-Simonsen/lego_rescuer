#!/usr/bin/env pybricks-micropython
# ---------------------------------------------------------------------------- #
#                                    IMPORT                                    #
# ---------------------------------------------------------------------------- #
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from folder_management2 import create_folder, save_data, ask_to_continue
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
    motor = Motor(Port.D)
    touch_sensor = TouchSensor(Port.S1)
    dist_sensor = UltrasonicSensor(Port.S2)
    sample_number = 0

    # Search settings
    search_angle = 60

    # Tests settings
    samples_per_location = 15
    n_locations = 6
    # ----------------------------- Folder management ---------------------------- #
    # If old_observation is a number it will continue from that observation
    old_observation = 0
    observation_name = "H_Slit"
    # Create a new folder
    foldername = create_folder(old_observation, obs_name=observation_name)

    # ----------------------------- Start the run ----------------------------- #
    n_samples = n_locations * samples_per_location
    print("[INFO] Press the button to start the run", file=sys.stderr)

    # Loop through samples, scan the can in front of the robot, store the data in a separate csv file and store in unique folder
    initial_angle = motor.angle()
    while sample_number < n_samples:

        if touch_sensor.pressed():
            for i in range(samples_per_location):
                print("---------------------- Run",
                      sample_number+1, " -----------------------\n")

                # ----- Sweap the area and sample -----
                motor.run_target(75, initial_angle)

                motor.run_angle(75, -search_angle)
                wait(200)

                # Rotate the robot at speed 15
                motor.run(30)

                # Sample data untill the robot has rotated search_angle degrees
                measurements = []
                while motor.angle() < initial_angle + search_angle:
                    if measurements == [] or motor.angle() != measurements[-1][0]:
                        measurements.append(
                            (motor.angle(), dist_sensor.distance()))
                    # print("data= ", measurements)
                motor.stop()
                # print("data= ", measurements)

                # Increament sample number
                sample_number = len(os.listdir(foldername)) + 1

                # ----- Save the data to a csv file -----
                save_data(measurements, foldername,
                          samples_per_location=samples_per_location)
                motor.run_target(75, initial_angle)
            print("[INFO] Press the button to start next run ")
            ev3.speaker.beep()
        else:
            wait(100)

    print("[INFO] All runs completed", file=sys.stderr)


if __name__ == "__main__":
    main()
