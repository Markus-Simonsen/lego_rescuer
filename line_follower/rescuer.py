from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from pybricks.hubs import EV3Brick


class Rescuer:
    # Initialize the EV3 brick.
    ev3 = EV3Brick()

    # Initialize the motors.
    gripper_motor = Motor(Port.A)
    right_motor = Motor(Port.B)
    left_motor = Motor(Port.C)

    # Sensors
    color_sensor_left = ColorSensor(Port.S2)
    color_sensor_right = ColorSensor(Port.S3)
    ultrasound = UltrasonicSensor(Port.S4)

    # Calculate the light threshold. Choose values based on your measurements.
    BLACK = 20
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    def follow_line:

        left_speed = 100
        right_speed = 100

        # Start following the line endlessly.
        while True:
            # if the left sensor sees the line, turn left
            if color_sensor_left.reflection() < threshold:
                left_speed = 0
            # if the right sensor sees the line, turn right
            elif color_sensor_right.reflection() < threshold:
                right_speed = 0
            else:
                left_speed = 100
                right_speed = 100

            print(
                "left: "
                + color_sensor_left.reflection()
                + "right: "
                + color_sensor_right.reflection()
            )

            # Set the motor speeds.
            left_motor.dc(left_speed)
            right_motor.dc(right_speed)