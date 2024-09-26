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
from pybricks.ev3devices import Motor, ColorSensor
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
    def __init__(self, kp, ki, kd):
        # Set PID values.
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Initialize the PID variables.
        self.integral = 0
        self.derivative = 0
        self.last_error = 0

        # Initialize the maximum speed.
        self.max_speed = 100
        self.velocity = 0

    def update(self, error):
        self.integral += error
        self.derivative = error - self.last_error
        self.last_error = error
        self.velocity = self.kp * error + self.ki * self.integral + self.kd * self.derivative

    def left_speed(self):
        if self.velocity > self.max_speed:
            return self.max_speed
        else:
            return self.velocity
        
    def right_speed(self):
        if self.velocity > self.max_speed:
            return self.max_speed
        else:
            return self.velocity
    
# ---------------------------------------------------------------------------- #
#                                   Functions                                  #
# ---------------------------------------------------------------------------- #
# Logger function for light sensor and motor values in .csv file
def log():
    with open(filename, "a") as file:
        # Get the current time in seconds.
        current_time = start_time - time.time()

        #log = "{}, {}, {}, {}, {}\n".format(current_time, line_sensor_left.reflection(), line_sensor_right.reflection(), left_motor.speed(), right_motor.speed())
        # Write the current time to the file.
        file.write(f"{current_time}, {line_sensor_left.reflection()}, {line_sensor_right.reflection()},{left_motor.stalled()}, {left_motor.speed()},{right_motor.stalled()}, {right_motor.speed()}\n")


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

# ---------------------------------- Logger ---------------------------------- #
# Start timer
start_time = time.time()

# Filename
filename = "data.csv"
with open(filename, "a") as file:
    file.write("Time, Left_sensor, Right_sensor,Left_motor_stall, Left_motor,Left_motor_stall, Right_motor\n")





# ---------------------------- PID Initialization ---------------------------- #
# Initialize the PID controller.
KP = 1
KI = 0
KD = 0

PID = PID_controller(1, 0, 0)

# ------------------------------ Light threshold ----------------------------- #
# Calculate the light threshold. Choose values based on your measurements.
BLACK = 20
WHITE = 85
threshold = (BLACK + WHITE) / 2

# # Set the drive speed at 100 millimeters per second.
DRIVE_SPEED = 100



# # For example, if the light value deviates from the threshold by 10, the robot
# steers at 10*1.2 = 12 degrees per second.
left_speed = 100
right_speed = 100

# ev3.speaker.play_file("/music.wav")

# # Start following the line endlessly.
while True:
   
    # Initialize the error.
    error = line_sensor_left.reflection() - line_sensor_right.reflection()

    # Update the PID controller.
    PID.update(error)
    # Set the motor speeds.
    left_motor.run()

    # You can wait for a short time or do other things in this loop.
    wait(50)
