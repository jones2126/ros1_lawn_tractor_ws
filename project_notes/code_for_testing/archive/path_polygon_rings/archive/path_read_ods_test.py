#!/usr/bin/env python
# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_read_ods_test.py
import pandas as pd

# Load the .ods file
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = folder_path + 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'

data = pd.read_excel(file_path, engine='odf', sheet_name='Obstacle')

# Rename the columns for clarity
data.columns = ['Reference', 'x', 'y', 'radius']

# Extract the data into separate variables
reference = data['Reference'].values
x = data['x'].values
y = data['y'].values
radius = data['radius'].values

# Print the extracted variables
print(reference)
print(x)
print(y)
print(radius)
