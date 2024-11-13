import os


def rename_files(directory):
    for filename in os.listdir(directory):
        if filename.endswith('.csv'):  # Change the file extension as needed
            name, ext = os.path.splitext(filename)
            if name[-2] == '_':
                new_name = name[:-2] + "_0" + name[-1] + ext
                os.rename(os.path.join(directory, filename),
                          os.path.join(directory, new_name))


# Specify the directory where the files are located
directory = 'Run_3'

rename_files(directory)
