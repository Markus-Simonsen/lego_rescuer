from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick

import PID as pid


class Rescuer:
    # Initialize the EV3 brick.
    ev3 = EV3Brick()

    # Initialize the motors.
    gripper_motor = Motor(Port.A)
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)

    # Initialize the color sensor.
    line_sensor_right = ColorSensor(Port.S1)
    touch_sensor = TouchSensor(Port.S2)
    light_sensor = LightSensor(Port.S3)
    line_sensor_left = ColorSensor(Port.S4)

    # ---------------------------- PID Initialization ---------------------------- #
    # Initialize the PID controller.
    KP = 30
    KI = 0
    KD = 2

    # # Set the drive speed at 100 millimeters per second.
    max_speed = 150
    base_speed = 90

    robot_pid_controller = pid.PID_controller(
        KP,
        KI,
        KD,
        base_speed,
        max_speed,
        line_sensor_left,
        line_sensor_right,
        left_motor,
        right_motor,
    )

    def activate_gripper(self):
        while self.touch_sensor.pressed() == False:
            self.left_motor.run(-self.base_speed)
            self.right_motor.run(-self.base_speed)
        self.left_motor.stop()
        self.right_motor.stop()
        self.gripper_motor.run_angle(100, -90)

    def turn_180(self):
        # Turn 180 degrees
        angles = 0

        # Turn speed
        speed = 100

        # Total angle
        degrees = 545

        # Run the motors
        while angles < degrees:
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
            angles = self.left_motor.angle()
            print(angles)

        # Stop the motors
        self.left_motor.run(0)
        self.right_motor.run(0)

    def grip_can(self):

        # Gripper
        grip_angle = -120
        grip_speed = 100

        while True:
            self.left_motor.run(- self.base_speed)  # Go reverse
            self.right_motor.run(- self.base_speed)  # Go reverse

            if self.touch_sensor.pressed():  # Touch sensor pressed
                self.left_motor.run(0)  # Stop the motors
                self.right_motor.run(0)  # Stop the motors
                self.gripper_motor.run_angle(
                    grip_speed, grip_angle)  # Close the gripper

                return True

    def behaviour_tree(self):
        # TODO: maybe implement state machine
        while (
            self.light_sensor.reflection() < 50
            and self.line_sensor_left.reflection() < 50
            and self.line_sensor_right.reflection() < 50
        ):
            self.robot_pid_controller.run()
        self.robot_pid_controller.run()
        self.turn_180()
        # self.robot_pid_controller.run()
