from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, TouchSensor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
import sys

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
robot.reset(angle=0.0) # Reset angle to 0

# Search settings
n_samples = 30
sample_number = 0
search_angle = 40


# Create unique folder
def create_folder(folder="run_1"):
    # Create a new folder called run_1, run_2, etc. based on the number of existing folders called run_1, run_2, etc.
    import os
    if os.path.exists(folder):
        i = 2
        while os.path.exists("run_" + str(i)):
            i += 1
        folder = "run_" + str(i)
    print("Creating folder: ", folder, file=sys.stderr)

    return folder

# Function to save the data to a csv file in unique folder
def save_data(data, filenumber, folder):
    # Save the data to a csv file
    with open(str(folder) + "/sample_" + str(filenumber) + ".csv", "w") as f:
        for row in data:
            f.write(str(row[0]) + "\t" + str(row[1]) + "\n")
    


# Create a unique folder
foldername = create_folder()

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
            measurements.append([robot.angle(), dist_sensor.distance()])
        robot.stop()
        print("data= ", measurements)

        # Increament sample number
        sample_number += 1 # Done before to have sample 1-30 instead of 0-29

        # ----- Save the data to a csv file -----
        save_data(measurements, sample_number, foldername)
        print("Run ", sample_number, " completed. Press the button to start next run", file=sys.stderr)

    else:
        wait(100)

print("All runs completed", file=sys.stderr)