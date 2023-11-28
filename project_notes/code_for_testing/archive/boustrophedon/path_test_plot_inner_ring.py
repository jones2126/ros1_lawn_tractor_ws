# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_plot_inner_ring.py

import matplotlib.pyplot as plt

# Reading the data
# space delimeted file, no header
#file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophen_inner_ring.txt'
# with open(file_path, 'r') as file:
#     lines = file.readlines()
# # Parsing the data
# x, y = [], []
# for line in lines:
#     values = line.split()
#     x.append(float(values[0]))
#     y.append(float(values[1]))

# comma deliminted file with a header
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_innermost_ring.csv'
with open(file_path, 'r') as file:
    # Skipping the header row
    next(file)
    # Parsing the data
    x, y = [], []
    for line in file:
        values = line.strip().split(',')
        x.append(float(values[0]))
        y.append(float(values[1]))
# Plotting as a line plot
plt.plot(x, y)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Line Plot of X and Y')
plt.show()