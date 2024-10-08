#!/usr/bin/env pybricks-micropython

"""
Example LEGO MINDSTORMS EV3 Robot Educator Color Sensor Down Program
----------------------------------------------------------------------

This program requires LEGO EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""
# ---------------------------------------------------------------------------- #
#                                    Import                                    #
# ---------------------------------------------------------------------------- #
from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase 
from pybricks.hubs import EV3Brick
import time


# ---------------------------------------------------------------------------- #
#                                    Classes                                   #
# ---------------------------------------------------------------------------- #


# --------------------------------- PID Class -------------------------------- #
class PID_controller:
    def __init__(self, kp, ki, kd,base_speed,max_speed,line_sensor_left,line_sensor_right,left_motor,right_motor):
        # Set PID values.
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Base speed
        self.base_speed = base_speed

        # Thresholds
        self.white_threshold = 15
        self.black_threshold = 6

        # Sensors
        self.line_sensor_left = line_sensor_left
        self.line_sensor_right = line_sensor_right

        # Motors
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Initialize the PID variables.
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        self.last_time = time.time()

        # Initialize the maximum speed.
        self.max_speed = max_speed
        self.left_speed = 50
        self.right_speed = 50
        self.output = 0

    def update(self, error):

        # Update the integral and derivative.
        self.integral += error
        self.derivative = error - self.last_error
        self.last_error = error

        # Calculate the output.
        self.output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.derivative / (time.time() - self.last_time)
        )


        # Cap speed
        if self.output > self.max_speed:
            self.output = self.max_speed
        elif self.output < -self.max_speed:
            self.output = -self.max_speed

        # Set the motor speeds.
        self.left_speed = self.base_speed + self.output
        self.right_speed = self.base_speed - self.output

        # Check for sharp turns
        if self.line_sensor_left.reflection() > self.white_threshold and self.line_sensor_right.reflection() < self.black_threshold:
            self.left_speed = self.base_speed*3
            self.right_speed = -self.base_speed*3
        elif self.line_sensor_left.reflection() < self.black_threshold and self.line_sensor_right.reflection() > self.white_threshold:
            self.left_speed = -self.base_speed*3
            self.right_speed = self.base_speed*3

        # Update the time.
        self.last_time = time.time()
    
    def print_values(self):
        # Print time
        print("--------------------")
        print("time: ", time.time() - self.last_time)
        # Print error
        print("left: ", self.line_sensor_left.reflection())
        print("right: ", self.line_sensor_right.reflection())
        print("error: ", self.last_error)
        print("output: ", self.output,end="\n")

    
    def run(self):
        
        # Initialize the error.
        error = line_sensor_left.reflection() - line_sensor_right.reflection()

        # Update the PID controller.
        self.update(error)

        # Print values
        self.print_values()

        # Set the motor speeds.
        left_motor.run(self.left_speed)
        right_motor.run(self.right_speed)
        


# ---------------------------------------------------------------------------- #
#                                   Functions                                  #
# ---------------------------------------------------------------------------- #
# Logger function for light sensor and motor values in .csv file
def log():
    with open(filename, "a") as file:
        # Get the current time in seconds.
        current_time = start_time - time.time()

        # log = "{}, {}, {}, {}, {}\n".format(current_time, line_sensor_left.reflection(), line_sensor_right.reflection(), left_motor.speed(), right_motor.speed())
        # Write the current time to the file.
        file.write(
            str(current_time)
            + ", "
            + str(line_sensor_left.reflection())
            + ", "
            + str(line_sensor_right.reflection())
            + ", "
            + str(left_motor.stalled())
            + ", "
            + str(left_motor.speed())
            + ", "
            + str(right_motor.stalled())
            + ", "
            + str(right_motor.speed())
            + "\n"
        )


# ---------------------------------------------------------------------------- #
#                                     Main                                     #
# ---------------------------------------------------------------------------- #

# ------------------------ Initialize brick and motors ----------------------- #
# Initialize the EV3 brick.
ev3 = EV3Brick()

# Initialize the motors.
right_motor = Motor(Port.C)
left_motor = Motor(Port.B)


# Initialize the color sensor.
line_sensor_left = ColorSensor(Port.S4)
line_sensor_right = ColorSensor(Port.S1)
light_sensor = LightSensor(Port.S3) 

# ---------------------------------- Logger ---------------------------------- #
# Start timer
start_time = time.time()

# Filename
filename = "data.csv"
with open(filename, "a") as file:
    file.write(
        "Time, Left_sensor, Right_sensor,Left_motor_stall, Left_motor,Left_motor_stall, Right_motor\n"
    )


# ---------------------------- PID Initialization ---------------------------- #
# Initialize the PID controller.
KP = 30
KI = 0
KD = 2


# ------------------------------ Light threshold ----------------------------- #
# Calculate the light threshold. Choose values based on your measurements.
BLACK = 20
WHITE = 85
threshold = (BLACK + WHITE) / 2

# # Set the drive speed at 100 millimeters per second.
max_speed = 150
base_speed = 90

PID = PID_controller(KP,KI,KD,base_speed,max_speed,line_sensor_left,line_sensor_right,left_motor,right_motor)
# -------------------------------- While Loop -------------------------------- #
# Start following the line endlessly.
while True:

    # PID controller
    PID.run()
    # Print light sensor values
    print("Left: ",light_sensor.reflection())

    # Log the data.
    log()

    # You can wait for a short time or do other things in this loop.
    wait(50)
