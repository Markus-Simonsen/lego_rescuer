import pandas as pd
import matplotlib.pyplot as plt


col_names = ["Index",
             "Time",
             "Left_sensor",
             "Light_sensor",
             "Right_sensor",
             "Triple_light",
             "Left_motor_speed",
             "Right_motor_speed",
             "Left_motor_angle",
             "Right_motor_angle"]
# Read the LOG.CSV file
df = pd.read_csv('log.csv', delimiter=',', names=col_names, skiprows=1)
# Time, Left_sensor, Light_sensor, Right_sensor, Triple_light, Left_motor_speed, Right_motor_speed, Left_motor_angle, Right_motor_angle


# Extract the data columns
time = df['Time']
left_sensor = df['Left_sensor']
light_sensor = df['Light_sensor']
right_sensor = df['Right_sensor']
triple_light = df['Triple_light']
left_motor_speed = df['Left_motor_speed']
right_motor_speed = df['Right_motor_speed']
left_motor_angle = df['Left_motor_angle']
right_motor_angle = df['Right_motor_angle']

# Create a new DataFrame with the extracted data columns
data = pd.DataFrame({
    'Time': time,
    'Left_sensor': left_sensor,
    'Light_sensor': light_sensor,
    'Right_sensor': right_sensor,
    'Triple_light': triple_light,
    'Left_motor_speed': left_motor_speed,
    'Right_motor_speed': right_motor_speed,
    'Left_motor_angle': left_motor_angle,
    'Right_motor_angle': right_motor_angle
})

# Print the new DataFrame
print(data)


# Calculate the error as the difference between the left and right sensor
error = left_sensor.astype(int) - light_sensor.astype(int)

# Print the error
# Print the error
print(error)

# Plot error over time
plt.plot(time, error)
# Plot line to show the zero error line
plt.axhline(y=0, color='r', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error over Time')
plt.show()

# Plot left and right motor speeds over time
plt.plot(time, error)
plt.plot(time, left_motor_speed, label='Left Motor Speed')
plt.plot(time, right_motor_speed, label='Right Motor Speed')
plt.xlabel('Time')
plt.ylabel('Motor Speed')
plt.title('Motor Speeds over Time')
plt.legend()
plt.show()
