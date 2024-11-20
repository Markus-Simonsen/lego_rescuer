import os
from os import path

order_i = 0
# ---------------------------------------------------------------------------- #
#                                   FUNCTIONS                                  #
# ---------------------------------------------------------------------------- #

# Log sample number with degrees in CSV


def get_folder_samples(folder_path):
    return len(os.listdir(folder_path))


def set_order_i(value):
    global order_i
    order_i = value
# ----------------------------- Create new folder ---------------------------- #


def create_folder(obs_continue=0, obs_name="observation"):
    # Make top folder
    folder_path = "data_V4"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    # Observation number
    if obs_continue:
        count = obs_continue  # Continue from last observation
    else:
        count = len(os.listdir(folder_path)) + 1  # Make new observation

    # Make observation folder
    folder_path = folder_path + "/"+obs_name+"_" + str(count)

    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    return folder_path

# --------------------------------- Save data -------------------------------- #
# Function to save the data to a csv file in unique folder


def save_data(data, folder_path, filename="sample", samples_per_location=5, locations=[30, 15, 0, -15, -30]):
    global order_i
    # Assertion: Data is a list of tuples
    assert type(data) == list, "Data must be a list of tuples"
    assert type(data[0]) == tuple, "Data must be a list of tuples"
    assert len(data) > 1, "Data must contain more than one element"

    print("[LOG]  Saving data to folder: ", folder_path)

    # Count samples
    dir_list = os.listdir(folder_path)

    count = len(dir_list) + 1

    count_str = '{0:03d}'.format(count)
    # Make sub count of 5 so it becomes 1.1, 1.2, 1.3, 1.4, 1.5

    # Make filename
    filename = folder_path + "/" + filename + "_" + \
        str(locations[order_i]) + "_" + count_str + ".csv"

    # Update order_i
    if count % samples_per_location == 0:
        order_i += 1

    # Write data to file
    with open(filename, "w") as file:
        file.write("Angle (deg), Distance (mm)\n")
        for row in data:
            file.write(str(row[0]) + "," + str(row[1]) + "\n")

    # Print log
    print("[LOG]  " + str(len(data)) +
          " elements logged to: " + filename)

    # Log descriptive statistics
    mean = sum([row[1] for row in data]) / len(data)
    mean = round(mean, 2)
    variance = sum([(row[1] - mean)**2 for row in data]) / len(data)
    variance = round(variance, 2)
    min_val = min([row[1] for row in data])
    max_val = max([row[1] for row in data])

    print("[DATA] Descriptive statistics: ")
    # print("[DATA]  Mean: ", mean, "Variance: ", variance,
    #      "Min value: ", min_val, "Max value: ", max_val)
    print("[DATA] Mean: ", mean)
    print("[DATA] Variance: ", variance)
    print("[DATA] Min value: ", min_val)
    print("[DATA] Max value: ", max_val)


# ------------------------------ Ask to continue ----------------------------- #


def ask_to_continue():
    obs_continue = input(
        "Make new observation? \nPress \"n\" to enter old observation number: ")
    if obs_continue == "n" or obs_continue == "N":
        obs_continue = int(input("Which observation should be continued? "))
    else:
        obs_continue = 0

    return obs_continue
