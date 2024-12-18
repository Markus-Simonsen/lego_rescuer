#!/usr/bin/env pybricks-micropython
# Question 1: Can we assume that the tomato can will be placed directly in front of the end of the line?
# Question 2: Ramp management, do we just film one take going up and one take going down?
# Question 3: Angle of can?
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
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
# import time


# ---------------------------------------------------------------------------- #
#                                    Classes                                   #
# ---------------------------------------------------------------------------- #


# --------------------------------- PID Class -------------------------------- #
class PID_controller:
    def __init__(
        self,
        kp,
        ki,
        kd,
        base_speed,
        max_speed,
        line_sensor_left,
        line_sensor_right,
        light_sensor,
        left_motor,
        right_motor,
        line_follower_calibration
    ):
        # Set PID values.
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Base speed
        self.base_speed = base_speed

        # Thresholds
        self.white_threshold = 15
        self.black_threshold = 6
        self.light_threshold = 35

        # Triple light
        self.triple_light_list = [0]
        self.triple_white = False
        self.triple_light_count = 0

        # Sensors
        self.line_sensor_left = line_sensor_left
        self.line_sensor_right = line_sensor_right
        self.light_sensor = light_sensor

        # Motors
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Initialize the PID variables.
        self.integral = 0
        self.derivative = 0
        self.last_error = 0
        # self.last_time = time.time()
        self.calibration = line_follower_calibration

        # Initialize the maximum speed.
        self.max_speed = max_speed
        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.left_speed = 50
        self.right_speed = 50
        self.output = 0

        self.sample_count = 0

        # Initialize the logger
        self.log_i = 0
        self.filename = "log.csv"
        # self.start_time = time.time()
        # with open(self.filename, "a") as file:
        #     file.write(
        #         "Index, Time, Left_sensor,Light_sensor, Right_sensor, Triple_light, Left_motor_speed, Right_motor_speed, Left_motor_angle, Right_motor_angle\n"
        #     )

    def log(self):
        with open(self.filename, "a") as file:
            # Get the current time in seconds.
            # current_time = int((time.time() - self.start_time)*100)/100

            # log = "{}, {}, {}, {}, {}\n".format(current_time, line_sensor_left.reflection(), line_sensor_right.reflection(), left_motor.speed(), right_motor.speed())
            # Write the current time to the file.
            file.write(
                str(self.log_i)
                + ", "
                # + str(current_time)
                + ", "
                + str(self.line_sensor_left.reflection())
                + ", "
                + str(self.light_sensor.reflection())
                + ", "
                + str(self.line_sensor_right.reflection())
                + ", "
                + str(self.triple_light_count)
                + ", "
                + str(self.left_motor.speed())
                + ", "
                + str(self.right_motor.speed())
                + ", "
                + str(self.left_motor.angle())
                + ", "
                + str(self.right_motor.angle())
                + "\n"
            )
            self.log_i += 1

    def abs(self, x):
        if x < 0:
            return -x
        return x

    def update(self, error):

        if abs(error) < 2:
            error = 0

        # Update the integral and derivative.
        self.integral += error
        self.derivative = error - self.last_error
        self.last_error = error

        # Calculate the output.
        self.output = (
            self.kp * error
            + self.ki * self.integral
            # (time.time() - self.last_time)
            + self.kd * self.derivative / 0.01
        )

        # Cap speed
        if self.output > self.max_speed:
            self.output = self.max_speed
        elif self.output < -self.max_speed:
            self.output = -self.max_speed

        self.left_speed = self.base_speed + self.output
        self.right_speed = self.base_speed - self.output
        # Set the motor speeds.
        if self.output > 0:
            self.left_speed = self.base_speed + self.output
            self.right_speed = self.base_speed - 1.5*self.output
        else:
            self.right_speed = self.base_speed + abs(self.output)
            self.left_speed = self.base_speed - 1.5*abs(self.output)

        # Check for sharp turns
        speed_diff = int(self.base_speed * 2.5)
        if (
            self.line_sensor_left.reflection() > self.white_threshold
            and self.line_sensor_right.reflection() < self.black_threshold
        ):
            self.left_speed = speed_diff
            self.right_speed = -speed_diff
        elif (
            self.line_sensor_left.reflection() < self.black_threshold
            and self.line_sensor_right.reflection() > self.white_threshold
        ):
            self.left_speed = -speed_diff
            self.right_speed = speed_diff

        # Update the time.
        # self.last_time = time.time()

    # def incline(self):

    def print_values(self):
        # Print time
        # Clear terminal
        # print("\033[9A", end="")
        print("--------------------")
        # print("time: ", time.time())
        # Print error
        print("left: ", self.line_sensor_left.reflection() + self.calibration)
        print("right: ", self.line_sensor_right.reflection())
        print("error: ", self.last_error)
        print("output: ", self.output, end="\n")

    def run(self):
        # Initialize the error.
        error = (
            self.line_sensor_left.reflection()
            - self.line_sensor_right.reflection()
            - self.calibration
        )

        # Update the PID controller.
        self.update(error)

        # Log tripple light
        # self.log_triple_light()

        # Print values
        # self.print_values()

        # Set the motor speeds.
        if int(self.prev_left_speed) != int(self.left_speed) or int(self.prev_right_speed) != int(self.right_speed):
            self.prev_left_speed = self.left_speed
            self.prev_right_speed = self.right_speed
            self.left_motor.run(self.left_speed)
            self.right_motor.run(self.right_speed)
        # else:
        # print("saved time!!!")
        # self.left_motor.run(self.left_speed)
        # self.right_motor.run(self.right_speed)
        # # Log the data.
        # self.log()


# ---------------------------------------------------------------------------- #
#                                   Functions                                  #
# ---------------------------------------------------------------------------- #
# Logger function for light sensor and motor values in .csv file


# ---------------------------------------------------------------------------- #
#                                     Main                                     #
# ---------------------------------------------------------------------------- #


# ------------------------ Initialize brick and motors ----------------------- #
def main():
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
    def init_logger(filename="data.csv"):

        with open(filename, "a") as file:
            file.write(
                "Time, Left_sensor, Right_sensor,Left_motor_stall, Left_motor,Left_motor_stall, Right_motor\n"
            )

    # ---------------------------- PID Initialization ---------------------------- #
    # Initialize the PID controller.
    KP = 40
    KI = 0
    KD = 5

    # ------------------------------ Light threshold ----------------------------- #

    # # Set the drive speed at 100 millimeters per second.
    max_speed = 500
    base_speed = 130

    # my_PID = PID_controller(
    #     KP,
    #     KI,
    #     KD,
    #     base_speed,
    #     max_speed,
    #     line_sensor_left,
    #     line_sensor_right,
    #     light_sensor,
    #     left_motor,
    #     right_motor,
    # )
    # -------------------------------- While Loop -------------------------------- #
    # Start following the line endlessly.
    light = []
    while True:
        pass

        # PID controller
        # my_PID.run()

        # Print light sensor values
        # light.append(light_sensor.reflection())
        # Average light sensor values
        # print("AVERAGE: ", sum(light) / len(light))

        # Log the data.


if __name__ == "__main__":
    main()
