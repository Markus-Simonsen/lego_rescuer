import os


def rename_files(directory):
    count = 0
    for filename in os.listdir(directory):
        if filename.endswith('.csv'):  # Change the file extension as needed
            name, ext = os.path.splitext(filename)

            new_name = name[-3:] + "_" + name[:-4] + "_deg" + ext
            os.rename(os.path.join(directory, filename),
                      os.path.join(directory, new_name))
            count += 1
            print(f"Renamed {filename} to {new_name}")
    print(f"Renamed {count} files in {directory}")


# Specify the directory where the files are located
directory = 'SciMet_data/H_Tupes'

rename_files(directory)
