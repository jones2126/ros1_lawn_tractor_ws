# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_plot_before_running_simulation.py
import matplotlib.pyplot as plt

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

# File paths
file_path1 = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophen_generator_output.txt'
file_path2 = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/ready_to_test/Site_01_rings_input_path.txt'

# Function to read data and return x and y coordinates
def read_data(file_path):
    x, y = [], []
    with open(file_path, 'r') as file:
        for line in file:
            values = line.split()
            x.append(float(values[0]))
            y.append(float(values[1]))
    return x, y

# Reading data from files
x1, y1 = read_data(file_path1)
x2, y2 = read_data(file_path2)

# Plotting
plt.plot(x1, y1, color='blue', label='File 1: Blue Segments')
plt.plot(x2, y2, color='green', label='File 2: Green Segments')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plot of Line Segments')
plt.legend()
plt.axis('equal')  # Ensure equal aspect ratio
plt.show()
