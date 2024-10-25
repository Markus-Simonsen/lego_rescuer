from pybricks.ev3devices import Motor, ColorSensor, TouchSensor, UltrasonicSensor
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick
import time

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
    ultrasonic_sensor = UltrasonicSensor(Port.S2)
    light_sensor = LightSensor(Port.S3)
    line_sensor_left = ColorSensor(Port.S4)

    # Triple Light
    white_threshold = 18
    light_threshold = 48

    triple_light_list = [0]
    triple_white = False
    triple_light_count = 0

    # Initialize the logger
    log_i = 0
    filename = "log.csv"
    start_time = time.time()
    with open(filename, "a") as file:
        file.write(
            "Time, Left_sensor,Light_sensor, Right_sensor, Triple_light, Left_motor_speed, Right_motor_speed, Left_motor_angle, Right_motor_angle\n"
        )

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



    def NOT(self):
        with open(self.filename, "a") as file:
            # Get the current time in seconds.
            current_time = int((time.time() - self.start_time)*100)/100
            log_i = self.log_i
            # log = "{}, {}, {}, {}, {}\n".format(current_time, line_sensor_left.reflection(), line_sensor_right.reflection(), left_motor.speed(), right_motor.speed())
            # Write the current time to the file.
            file.write(
                str(log_i)
                + ", "
                + str(current_time)
                + ", "
                + str(self.line_sensor_left.reflection())
                + ", "
                + str(self.line_sensor_right.reflection())
                + ", "
                + str(self.left_motor.stalled())
                + ", "
                + str(self.left_motor.speed())
                + ", "
                + str(self.right_motor.stalled())
                + ", "
                + str(self.right_motor.speed())
                + ", "
                + str(self.light_sensor.reflection())
                + ", "
                + str(self.triple_light_list)
                + "\n"
            )
            self.log_i += 1


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
            # print("[TRIPLE LIGHT] ", self.triple_light_list)
            if self.triple_white == False:  # If not already white
                self.triple_white = True
                self.triple_light_list.append(1)
            else:  # If already white
                self.triple_light_list[-1] += 1
        else:
            self.triple_white = False

    def triple_light(self, max_count=15):
        # Print threshold
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

    def abs(self, x):
        if x < 0:
            return -x
        return x

    def turn_angle(self, angle):
        degree_180 = 472
        angle = degree_180/180*angle
        #save start angle
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        angles = 0
        self.left_motor.run(0)
        self.right_motor.run(0)
        # Turn speed
        speed = 100

        # Total angle
        degrees = angle
        print("Turn:", angle)

        if angle > 0:
            while angles < degrees:

                self.left_motor.run(speed)
                self.right_motor.run(-speed)
                # the right motor is negative because it rotates opposite
                angles = (self.left_motor.angle()-left_start -(
                          self.right_motor.angle()-right_start)) / 2
                # print(angles)
        elif angle < 0:
            while angles < abs(degrees):

                self.left_motor.run(-speed)
                self.right_motor.run(speed)
                # the left motor is negative because it rotates opposite
                angles = (self.right_motor.angle()-right_start -(
                          self.left_motor.angle()-left_start)) / 2
                # print(angles)

        # Stop the motors
        self.left_motor.run(0)
        self.right_motor.run(0)
        wait(1000)

    def turn_180(self, degrees=472):
        # Turn 180 degrees
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        angles = 0
        self.left_motor.run(0)
        self.right_motor.run(0)
        # Turn speed
        speed = 100

        # Total angle
        degrees_180 = angles + degrees
        print("[TURN 180]")
        # Run the motors
        while angles < degrees_180:

            self.left_motor.run(speed)
            self.right_motor.run(-speed)
            angles = (self.left_motor.angle()-left_start +
                      right_start-self.right_motor.angle()) / 2
            # print(angles)

        # Stop the motors
        self.left_motor.run(-speed)
        self.right_motor.run(-speed)
        wait(1000)


    def scan_180(self, degrees=472):
        # Turn 180 degrees
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        angles = 0
        self.left_motor.run(0)
        self.right_motor.run(0)
        # Turn speed
        speed = 50

        # Total angle
        degrees_180 = angles + degrees
        print("[TURN 180]")
        # Run the motors
        # array to store the readings at each interval of 5 angles in the 180 degree turn
        angle_readings = []
        angles_prev = 0
        while angles < degrees_180:

            self.left_motor.run(speed)
            self.right_motor.run(-speed)
            angles_prev = angles
            angles = (self.left_motor.angle()-left_start +
                      right_start-self.right_motor.angle()) / 2
            # store angles readings from the ultrasound sensor
            if angles % 5 == 0 and angles != angles_prev:
                angle_readings.append(self.ultrasonic_sensor.distance())
            # print(angles)

        print(angle_readings)

        # return to the minimum distance reading
        min_distance = min(angle_readings)
        print(min_distance)
        # indices of min value
        min_indices = [index for index, value in enumerate(angle_readings) if value == min_distance]
        # find middle index of min indices
        index = min_indices[len(min_indices)//2]
        # index of min value from the other end of the list
        index = len(angle_readings) - index
        angles = 0
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        while angles < index*5:
            self.left_motor.run(-speed)
            self.right_motor.run(speed)
            angles = (-self.left_motor.angle()+left_start -
                      right_start+self.right_motor.angle()) / 2
            # print(angles)
        
        self.left_motor.run(0)
        self.right_motor.run(0)
        wait(1000)
        while self.ultrasonic_sensor.distance() > 45w:
            print(self.ultrasonic_sensor.distance())
            self.left_motor.run(-self.base_speed)
            self.right_motor.run(-self.base_speed)
        self.left_motor.run(0)
        self.right_motor.run(0)



    def grip_can(self):

        # Gripper
        grip_angle = -120
        grip_speed = 100
        self.gripper_motor.run_angle(
                    grip_speed, grip_angle)  # Close the gripper


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
            # Log
            # print("Left: ", self.line_sensor_left.reflection(),
            #       "Middle: ", self.light_sensor.reflection(),
            #       "Right: ", self.line_sensor_right.reflection(), end="\r")
        # Reverse
        self.turn_angle(90)
        self.scan_180()
        self.grip_can()
        while True:
            self.robot_pid_controller.run()


def main():
    jerry = Rescuer()
    while True:
        jerry.behaviour_tree()


if __name__ == "__main__":
    main()
