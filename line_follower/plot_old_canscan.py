import numpy as np
import matplotlib.pyplot as plt

log_data = np.genfromtxt('SciMet/sample_2.csv', delimiter=',')
angle_readings = log_data[1:, 1]

min_distance = min(angle_readings)
print(min_distance)
# indices of min value
min_indices = [index for index, value in enumerate(
    angle_readings) if value == min_distance]
# find middle index of min indices
index = min_indices[len(min_indices)//2]


angles = log_data[1:, 0]
distances = log_data[1:, 1]
plt.plot(angles, distances, 'ro')
plt.plot(angle_readings)
plt.show()
