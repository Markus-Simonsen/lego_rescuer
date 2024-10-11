from pybricks.ev3devices import Motor, ColorSensor, TouchSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick

import PID as PID


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

    # Triple Light
    # Thresholds
    white_threshold = 18
    light_threshold = 48

    # Triple light
    triple_light_list = [0]
    triple_white = False
    triple_light_count = 0

    # ---------------------------- PID Initialization ---------------------------- #
    # Initialize the PID controller.
    KP = 40
    KI = 0
    KD = 5

    # # Set the drive speed at 100 millimeters per second.
    max_speed = 500
    base_speed = 130

    print("PID Controller Initialized")
    robot_pid_controller = PID.PID_controller(
        KP,
        KI,
        KD,
        base_speed,
        max_speed,
        line_sensor_left,
        line_sensor_right,
        light_sensor,
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

    def log_triple_light(self):

        # Check for triple light
        if (self.light_sensor.reflection() > self.light_threshold and
            self.line_sensor_right.reflection() > self.white_threshold and
                self.line_sensor_left.reflection() > self.white_threshold):
            print("[TRIPLE LIGHT] ", self.triple_light_list)
            if self.triple_white == False:  # If not already white
                self.triple_white = True
                self.triple_light_list.append(1)
            else:  # If already white
                self.triple_light_list[-1] += 1
        else:
            self.triple_white = False

    def triple_light(self, max_count=15):
        # Print threshold
        print("Threshold: ", self.light_threshold)
        print("White Threshold: ", self.white_threshold)
        # Check for triple light
        if (self.light_sensor.reflection() > self.light_threshold and
            self.line_sensor_right.reflection() > self.white_threshold and
                self.line_sensor_left.reflection() > self.white_threshold):
            # Print reflections
            # print("Left: ", self.line_sensor_left.reflection(),
            #       "Middle: ", self.light_sensor.reflection(),
            #       "Right: ", self.line_sensor_right.reflection(), end="\r")
            print("[TRIPLE LIGHT] ", self.triple_light_count)

            if self.triple_white == False:  # If not already white
                self.triple_white = True
                self.triple_light_count = 1
            else:  # If already white
                self.triple_light_count += 1
        else:
            self.triple_white = False

        if self.triple_light_count > max_count:
            print("[TRIPLE LIGHT] ", self.triple_light_count)
            return True
        else:
            return False

    def turn_180(self):
        # Turn 180 degrees
        angles = 0
        self.left_motor.run(0)
        self.right_motor.run(0)
        # Turn speed
        speed = 100

        # Total angle
        degrees = 545
        print("[TURN 180]")
        # Run the motors
        while angles < degrees:

            self.left_motor.run(speed)
            self.right_motor.run(-speed)
            angles = self.left_motor.angle()  # + self.right_motor.angle()) / 2
            print(angles)

        # Stop the motors
        self.left_motor.run(0)
        self.right_motor.run(0)

    def grip_can(self):

        # Gripper
        grip_angle = -120
        grip_speed = 100

        # Motor speed
        hammer_speed = 500
        print("[GRIP CAN]")
        while True:
            self.left_motor.run(- hammer_speed)  # Go reverse
            self.right_motor.run(- hammer_speed)  # Go reverse

            if self.touch_sensor.pressed():  # Touch sensor pressed
                self.left_motor.run(0)  # Stop the motors
                self.right_motor.run(0)  # Stop the motors
                self.gripper_motor.run_angle(
                    grip_speed, grip_angle)  # Close the gripper

                return True

    def behaviour_tree(self):
        print("Behaviour Tree")
        # Print sensor values
        # while True:
        #     print("Left: ", self.line_sensor_left.reflection(),
        #           "Right: ", self.line_sensor_right.reflection(),
        #           "Middle: ", self.light_sensor.reflection())
        #     wait(200)
        # TODO: maybe implement state machine
        while (not self.triple_light(35)):
            self.robot_pid_controller.run()
            print("Left: ", self.line_sensor_left.reflection(),
                  "Middle: ", self.light_sensor.reflection(),
                  "Right: ", self.line_sensor_right.reflection(), end="\r")
        # Reverse
        self.left_motor.run(-self.base_speed/2)
        self.right_motor.run(-self.base_speed/2)
        wait(200)
        self.turn_180()
        # self.grip_can()
        # self.robot_pid_controller.run()


def main():
    jerry = Rescuer()
    while True:
        jerry.behaviour_tree()


if __name__ == "__main__":
    main()
