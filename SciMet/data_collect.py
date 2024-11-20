#!/usr/bin/env pybricks-micropython
# ---------------------------------------------------------------------------- #
#                                    IMPORT                                    #
# ---------------------------------------------------------------------------- #
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from data_management import create_folder, save_data, ask_to_continue, get_folder_samples, set_order_i
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
    confirm_touch = TouchSensor(Port.S3)

    # ------------------------------ Calibrate zero ------------------------------ #
    initial_angle = motor.angle()
    motor.run_target(75, initial_angle-15)

    print("Intial angle: ", motor.angle())
    angle = motor.angle()
    print("Press the touch sensor to calibrate the zero position")
    ev3.speaker.beep()
    while confirm_touch.pressed() == False:
        if touch_sensor.pressed():
            angle += 1
            print("Angle: ", angle)
            motor.run_target(75, angle)
        wait(100)
    ev3.speaker.beep()

    print("Zero position calibrated at:", motor.angle(), "deg")
    motor.reset_angle(0)

    # Search settings
    search_angle = 60
    search_speed = 75

    # Tests settings
    samples_per_location = 125
    locations = [0]
    # ----------------------------- Folder management ---------------------------- #
    # If old_observation is a number it will continue from that observation
    old_observation = 0
    observation_name = "H_tubes_narrowV2"
    # Create a new folder
    foldername = create_folder(old_observation, obs_name=observation_name)
    sample_number = get_folder_samples(foldername)
    set_order_i(sample_number // samples_per_location)

    # ----------------------------- Start the run ----------------------------- #
    n_samples = len(locations) * samples_per_location
    print("[INFO] Place the can at " +
          str(locations[sample_number // samples_per_location]) + " deg.")
    print("[INFO] Press the button to start the run", file=sys.stderr)

    # Loop through samples, scan the can in front of the robot, store the data in a separate csv file and store in unique folder
    initial_angle = motor.angle()
    while sample_number < n_samples:

        if touch_sensor.pressed():
            for i in range(samples_per_location):
                print("---------------------- Run",
                      sample_number+1, " -----------------------\n")

                # ----- Sweep the area and sample -----
                motor.run_target(search_speed, initial_angle)

                motor.run_angle(search_speed, -search_angle)
                wait(200)

                # Rotate the robot at speed 15
                motor.run(50)

                # Sample data untill the robot has rotated search_angle degrees
                measurements = []
                while motor.angle() < initial_angle + search_angle:
                    if measurements == [] or motor.angle() != measurements[-1][0]:
                        measurements.append(
                            (motor.angle(), dist_sensor.distance()))
                motor.stop()
                # print("data= ", measurements)

                # Increment sample number
                sample_number = len(os.listdir(foldername)) + 1

                # ----- Save the data to a csv file -----
                save_data(measurements, foldername,
                          samples_per_location=samples_per_location, locations=locations)
                motor.run_target(search_speed, initial_angle)
                if sample_number == n_samples:
                    ev3.speaker.beep()
            print("----------------------------------------------------")
            print("[INFO] ", locations[sample_number // samples_per_location - 1],
                  "deg sample completed", file=sys.stderr)
            print("----------------------------------------------------\n")
            print("[INFO] Place the can at " +
                  str(locations[sample_number // samples_per_location]) + " deg.")
            print("[INFO] Press the button to start next run ")

            ev3.speaker.beep()
        else:
            wait(100)

    print("[INFO] All runs completed", file=sys.stderr)


if __name__ == "__main__":
    main()
