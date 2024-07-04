#!/usr/bin/env python

'''
Read the data from the dubins path data and plot the points adding different colors based on look ahead.  Also plot circles
representing the obstacle so to confirm the starting point of the u-turns is adequate.

$ python3 ~ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_plot_dubins.py
'''
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# Function to read data from an Excel sheet
def read_data(file_path, sheet_name):
    df = pd.read_excel(file_path, sheet_name=sheet_name, header=0)
    df.columns = ['x', 'y', 'theta', 'lookahead', 'speed']
    points = df[['x', 'y', 'lookahead']].values.tolist()
    return points

# Function to plot the data
def plot_data(points):
    for i in range(len(points) - 1):
        x1, y1, lookahead1 = points[i]
        x2, y2, lookahead2 = points[i + 1]
        # Choose color based on lookahead value
        color = 'red' if lookahead1 == 2.5 else 'blue'
        # Plotting the line segment
        plt.plot([x1, x2], [y1, y2], color=color)

    # Add circles
    circle_center = (-10.52, -33.01)
    circle_radii = [6, 3.12, 2.4]  # U-turn start, Inflated, Best Fit
    circle_colors = ['green', 'orange', 'purple']
    circle_labels = ['Original (r=6)', 'Inflated (r=3.12)', 'Best Fit (r=2.4)']

    for radius, color, label in zip(circle_radii, circle_colors, circle_labels):
        circle = plt.Circle(circle_center, radius, fill=False, color=color, label=label)
        plt.gca().add_artist(circle)

    # Set labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Line Segments with Different Colors Based on Lookahead\nand Multiple Circles')
    plt.axis("equal")
    
    # Add legend
    plt.legend()

    # Show the plot
    plt.show()

# Main
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
sheet_name = 'path_dubins'
data_points = read_data(file_path, sheet_name)
plot_data(data_points)