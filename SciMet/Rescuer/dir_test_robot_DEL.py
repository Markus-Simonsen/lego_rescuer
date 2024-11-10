from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
import os


def log(data, folder_path, filename="sample"):
    # Assertion: Data is a list of tuples
    assert type(data) == list, "Data must be a list of tuples"
    assert type(data[0]) == tuple, "Data must be a list of tuples"

    # Count samples
    count = len(os.listdir(folder_path)) + 1

    # Make filename
    filename = folder_path + "/" + filename + "_" + str(count) + ".csv"

    # Write data to file
    with open(filename, "w") as file:
        file.write("Angle (deg), Distance (mm)\n")
        for i in range(len(data)):
            file.write(str(data[i][0]) + "," + str(data[i][1]) + "\n")

    # Print log
    print("[LOG] " + str(len(data)) + " elements logged to: " + filename)


def log_init(obs_continue=0, _filename="sample"):

    # Make top folder
    folder_path = "data"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Observation number
    if obs_continue:
        count = obs_continue  # Continue from last observation
    else:
        count = len(os.listdir(folder_path)) + 1  # Make new observation

    # Make observation folder
    folder_path = folder_path + "/observation_" + str(count)

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    return folder_path


def main():
    folder_path = log_init()

    data = [(2, 2), (3, 4), (5, 6)]
    log(data, folder_path)
    log(data, folder_path)
    log(data, folder_path)

    log(data, folder_path)
    log(data, folder_path)


if __name__ == "__main__":
    main()
