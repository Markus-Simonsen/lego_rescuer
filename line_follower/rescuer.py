from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick

import PID as pid


class Rescuer:
    # Initialize the EV3 brick.
    ev3 = EV3Brick()

    # Initialize the motors.
    right_motor = Motor(Port.C)
    left_motor = Motor(Port.B)
    gripper_motor = Motor(Port.A)

    # Initialize the color sensor.
    line_sensor_left = ColorSensor(Port.S4)
    line_sensor_right = ColorSensor(Port.S1)
    light_sensor = LightSensor(Port.S3)
    touch_sensor = TouchSensor(Port.S2)

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

    def do_180(self):
        self.left_motor.run(self.base_speed)
        self.right_motor.run(-self.base_speed)
        wait(1000)

    def grip_can(self):
        self.left_motor.run(-self.base_speed)
        self.right_motor.run(-self.base_speed)
        wait(1000)
        self.do_180()
        self.activate_gripper()

    def behaviour_tree(self):
        # TODO: maybe implement state machine
        while (
            self.light_sensor.reflection() < 50
            and self.line_sensor_left.reflection() < 50
            and self.line_sensor_right.reflection() < 50
        ):
            self.robot_pid_controller.run()
        self.robot_pid_controller.run()
        self.do_180()
        # self.robot_pid_controller.run()
