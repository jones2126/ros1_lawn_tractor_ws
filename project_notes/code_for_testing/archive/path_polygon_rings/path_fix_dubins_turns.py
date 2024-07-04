#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .xlsx file that represent the bounds around an obstacle and calculates the center 
point and radius. This will be used as input to additional path planning scripts.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_fix_dubins_turns.py
'''
import matplotlib.pyplot as plt
import pandas as pd

# Function to read data from an Excel sheet
def read_data(file_path, sheet_name):
    df = pd.read_excel(file_path, sheet_name=sheet_name, header=0)
    df.columns = ['x', 'y', 'theta', 'lookahead', 'speed']
    points = df[['x', 'y', 'lookahead']].values.tolist()
    return points

# Function to plot the data
def plot_data(points, circle_center, radius1, radius2):
    fig, ax = plt.subplots()

    for i in range(len(points) - 1):
        x1, y1, lookahead1 = points[i]
        x2, y2, lookahead2 = points[i + 1]

        # Choose color based on lookahead value
        color = 'red' if lookahead1 == 2.5 else 'blue'

        # Plotting the line segment
        ax.plot([x1, x2], [y1, y2], color=color)

    # Add circles to the plot
    circle1 = plt.Circle(circle_center, radius1, color='green', fill=False, linestyle='--', label='Radius 2.4')
    circle2 = plt.Circle(circle_center, radius2, color='purple', fill=False, linestyle='-', label='Radius 3.14')

    ax.add_artist(circle1)
    ax.add_artist(circle2)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Line Segments with Different Colors Based on Lookahead')
    ax.set_aspect('equal', adjustable='datalim')
    ax.legend()
    plt.grid(True)
    plt.show()

# Main
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
sheet_name = 'path_dubins'
data_points = read_data(file_path, sheet_name)
circle_center = (-10.52, -33.01)
radius1 = 2.4
radius2 = 3.14

plot_data(data_points, circle_center, radius1, radius2)
