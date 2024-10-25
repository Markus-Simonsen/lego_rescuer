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
    white_threshold = 17
    light_threshold = 45  # 48 før

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

    # Calibration
    line_follower_calibration = 1.4

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
        line_follower_calibration
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
        """
        while self.touch_sensor.pressed() == False:
            self.left_motor.run(-self.base_speed)
            self.right_motor.run(-self.base_speed)
        self.left_motor.stop()
        self.right_motor.stop()
        self.gripper_motor.run_angle(100, -90)
        """
        return True

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
# ------------------------------- Triple Light ------------------------------- #

    def triple_light(self, max_count=15):
        # Print threshold
        # Check for triple light
        if (self.light_sensor.reflection() > self.light_threshold and
            self.line_sensor_right.reflection() > self.white_threshold and
                self.line_sensor_left.reflection() > self.white_threshold):
            # Print reflections
            # print("Left: ", self.line_sensor_left.reflection(),
            #      "Middle: ", self.light_sensor.reflection(),
            #      "Right: ", self.line_sensor_right.reflection())
            print("[TRIPLE LIGHT] ", self.triple_light_count)

            if self.triple_white == False:  # If not already white
                self.triple_white = True
                self.triple_light_count = 1
            else:  # If already white
                self.triple_light_count += 1
        else:
            self.triple_white = False

        if self.triple_light_count >= max_count:
            print("[TRIPLE LIGHT] ", self.triple_light_count)
            return True
        else:
            return False

    def abs(self, x):
        if x < 0:
            return -x
        return x

# -------------------------------- Turn angle -------------------------------- #
    def turn_angle(self, angle, degree_180=494):
        print("Turn:", angle)
        angle = degree_180/180*angle
        # save start angle
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        angles = 0
        self.left_motor.run(0)
        self.right_motor.run(0)
        # Turn speed
        speed = 100

        # Total angle
        degrees = angle

        if angle > 0:
            while angles < degrees:

                self.left_motor.run(speed)
                self.right_motor.run(-speed)
                # the right motor is negative because it rotates opposite
                angles = (self.left_motor.angle()-left_start - (
                          self.right_motor.angle()-right_start)) / 2
                # print(angles)
        elif angle < 0:
            while angles < abs(degrees):

                self.left_motor.run(-speed)
                self.right_motor.run(speed)
                # the left motor is negative because it rotates opposite
                angles = (self.right_motor.angle()-right_start - (
                          self.left_motor.angle()-left_start)) / 2
                # print(angles)

        # Stop the motors
        self.left_motor.run(0)
        self.right_motor.run(0)
        wait(200)

# --------------------------------- Turn 180 --------------------------------- #
    def turn_180(self, degrees=494):
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

# --------------------------------- Scan 180 --------------------------------- #
    def can_scan(self, angle=180):
        # Scan Settings
        speed = 50
        mod_sample = 1
        grip_distance = 38

        # Turn 180 degrees
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        angles = 0
        self.left_motor.run(0)
        self.right_motor.run(0)

        # Total angle
        degrees_180 = 494
        degrees = degrees_180/180*angle

        print("[CAN SCAN]")
        # Run the motors
        # array to store the readings at each interval of 5 angles in the 180 degree turn
        angle_readings = []
        angles_prev = 0
        while angles < degrees:

            self.left_motor.run(speed)
            self.right_motor.run(-speed)
            angles_prev = angles
            angles = (self.left_motor.angle()-left_start +
                      right_start-self.right_motor.angle()) / 2
            # store angles readings from the ultrasound sensor
            if angles % mod_sample == 0 and angles != angles_prev:
                angle_readings.append(self.ultrasonic_sensor.distance())
            # print(angles)
        print(len(angle_readings))
        # print(angle_readings)

        # return to the minimum distance reading
        min_distance = min(angle_readings)
        print(min_distance)
        # indices of min value
        min_indices = [index for index, value in enumerate(
            angle_readings) if value == min_distance]
        # find middle index of min indices
        index = min_indices[len(min_indices)//2]
        # index of min value from the other end of the list
        print(index)
        index = len(angle_readings) - index

        angles = 0
        left_start = self.left_motor.angle()
        right_start = self.right_motor.angle()
        # ------------------------------- SOFTWARE FIX ------------------------------- #
        if angle == 80:
            software_fix = 15
        else:
            software_fix = 0
        # ------------------------------- SOFTWARE FIX ------------------------------- #
        while angles < index*mod_sample+software_fix:  # SOFTWARE FIX
            self.left_motor.run(-speed)
            self.right_motor.run(speed)
            angles = (-self.left_motor.angle()+left_start -
                      right_start+self.right_motor.angle()) / 2
            # print(angles)

        self.left_motor.run(0)
        self.right_motor.run(0)
        wait(200)
        return min_distance

    def check_turn_calibration(self, degrees_180=494, delay=700):

        self.turn_angle(360, degrees_180)
        wait(delay)
        self.turn_angle(360, degrees_180)
        wait(delay)
        self.turn_angle(90, degrees_180)
        wait(delay)
        self.turn_angle(90, degrees_180)
        wait(delay)
        self.turn_angle(90, degrees_180)
        wait(delay)
        self.turn_angle(90, degrees_180)
        wait(delay)
        self.turn_angle(-90, degrees_180)
        wait(delay)
        self.turn_angle(-90, degrees_180)
        wait(delay)
        self.turn_angle(-90, degrees_180)
        wait(delay)
        self.turn_angle(-90, degrees_180)
        wait(delay*4)
        self.turn_angle(-180, degrees_180)
        wait(delay)
        self.turn_angle(180, degrees_180)

    def approach_can(self):
        # Approach the can
        print("[APPROACH CAN]")
        approach_dist = 120

        while self.ultrasonic_sensor.distance() > approach_dist:
            # print(self.ultrasonic_sensor.distance())
            self.left_motor.run(-self.base_speed)
            self.right_motor.run(-self.base_speed)
        print("Distance: ", self.ultrasonic_sensor.distance())
        self.left_motor.run(0)
        self.right_motor.run(0)

    def touch_can(self, grip_distance=38):
        print("[TOUCH CAN]")
        grip_distance = 300

        while self.ultrasonic_sensor.distance() < grip_distance:
            # print(self.ultrasonic_sensor.distance())
            self.left_motor.run(-self.base_speed)
            self.right_motor.run(-self.base_speed)

            print("Distance: ", self.ultrasonic_sensor.distance())
        self.left_motor.run(0)
        self.right_motor.run(0)

# --------------------------------- Grip Can --------------------------------- #
    def grip_can(self):

        # Gripper
        grip_angle = -100
        grip_speed = 100
        self.gripper_motor.run_angle(
            grip_speed, grip_angle)  # Close the gripper

# -------------------------- Calibrate Line Follower ------------------------- #
    def calibrate_line_follower(self, samples=1000, stop=False):
        print("[CALIBRATION]: Beginning calibration")

        # Go straight
        if stop:
            self.left_motor.run(0)
            self.right_motor.run(0)
        else:
            self.left_motor.run(self.base_speed)
            self.right_motor.run(self.base_speed)

        # Collect samples
        sensor_values = []

        for i in range(samples):
            sensor_values.append([self.line_sensor_left.reflection(
            ), self.light_sensor.reflection(), self.line_sensor_right.reflection()])

        # Stop
        self.left_motor.run(0)
        self.right_motor.run(0)

        left_sensor_values = [x[0] for x in sensor_values]
        middle_sensor_values = [x[1] for x in sensor_values]
        right_sensor_values = [x[2] for x in sensor_values]

        # Calculate mean
        left_mean = sum(left_sensor_values)/samples
        middle_mean = sum(middle_sensor_values)/samples
        right_mean = sum(right_sensor_values)/samples

        left_calibration = left_mean - right_mean

        # Return the calibration value

        print("Left: ", left_mean)
        print("Middle: ", middle_mean)
        print("Right: ", right_mean)
        print("Left Calibration: ", left_calibration)
        print("Sensor values: ", sensor_values[:50])
        print("[CALIBRATION]: Calibration complete")

    def back_to_black(self, color_threshold=20, light_threshold=50, print_values=False):
        print("[BACK TO BLACK]")
        # Reverse

        # Wait for black
        while (self.line_sensor_left.reflection() > color_threshold and
               self.line_sensor_right.reflection() > color_threshold and
               self.light_sensor.reflection() > light_threshold):
            self.left_motor.run(self.base_speed)
            self.right_motor.run(self.base_speed)
            if print_values:
                print("[BLACK SEARCH]")
                print("Left: ", self.line_sensor_left.reflection(),
                      "Middle: ", self.light_sensor.reflection(),
                      "Right: ", self.line_sensor_right.reflection())
            pass
        self.left_motor.run(0)
        self.right_motor.run(0)

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
        self.turn_angle(180)
        self.back_to_black(print_values=False)
        # Search in a cone
        self.turn_angle(-40)
        self.can_scan(80)

        self.approach_can()
        self.turn_angle(-30)
        self.can_scan(60)
        self.touch_can()
        self.grip_can()
        while True:
            self.robot_pid_controller.run()


def main():
    jerry = Rescuer()
    # jerry.calibrate_line_follower()
    while True:
        jerry.behaviour_tree()


if __name__ == "__main__":
    main()
