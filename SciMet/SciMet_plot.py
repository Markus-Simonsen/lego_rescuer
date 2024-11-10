import numpy as np
import os
import matplotlib.pyplot as plt


folder_path = 'Data'
file_names = os.listdir(folder_path)


for file_name in file_names:
    log_data = np.genfromtxt(folder_path + "/"+file_name, delimiter=',')
    angles = log_data[1:, 0]
    distances = log_data[1:, 1]
    plt.title(file_name)
    plt.plot(angles, distances)
    plt.show()
