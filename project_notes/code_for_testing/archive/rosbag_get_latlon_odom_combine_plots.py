import pandas as pd
import matplotlib.pyplot as plt

# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_get_latlon_odom_combine_plots.py

# File paths
file_path_a='/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveA.csv'
file_path_b='/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveB.csv'

# Read data from CSV files
data_a = pd.read_csv(file_path_a)
data_b = pd.read_csv(file_path_b)

# Plotting the data
plt.figure(figsize=(10, 5))
# plt.plot(data_a['X'], data_a['Y'], 'ro', markersize=3, label='Drive A')
# plt.plot(data_b['X'], data_b['Y'], 'bo', markersize=3, label='Drive B')

plt.plot(data_a['X'].to_numpy(), data_a['Y'].to_numpy(), 'ro', markersize=3, label='Drive A')
plt.plot(data_b['X'].to_numpy(), data_b['Y'].to_numpy(), 'bo', markersize=3, label='Drive B')


plt.title('X and Y Coordinates')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.show()
