#!/usr/bin/env python

'''
Read the data from the specified sheet, and create Dubins path u-turns.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_plot3.py
'''
import matplotlib.pyplot as plt
import pandas as pd

max_records = 30

def read_data(file_path, sheet_name):
    df = pd.read_excel(file_path, sheet_name=sheet_name, header=None)
    points = df.iloc[:, [0, 1]].values.tolist()  # Select columns A and B
    return points[:max_records]

def plot_data(points1, points2):
    # Plot points from 'coverage_path_refrmttd' in blue
    x1, y1 = zip(*points1)
    plt.plot(x1, y1, color='blue', label='coverage_path_refrmttd')

    # Plot points from 'path_with_angle' in red
    x2, y2 = zip(*points2)
    plt.plot(x2, y2, color='red', label='path_with_angle')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'First {max_records} Points from Both Sheets')
    plt.legend()
    plt.axis("equal")
    plt.show()

# Main
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'

# Read data from both sheets
coverage_points = read_data(file_path, 'coverage_path_refrmttd')
angle_points = read_data(file_path, 'path_with_angle')

# Plot the data
plot_data(coverage_points, angle_points)