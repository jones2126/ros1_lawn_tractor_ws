#!/usr/bin/env python

'''
Read the data from the specified sheet, and create Dubins path u-turns.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_plot2.py
'''

import matplotlib.pyplot as plt
import pandas as pd

max_records = 700

def read_data(file_path, sheet_name):
    df = pd.read_excel(file_path, sheet_name=sheet_name, header=0)
    df.columns = ['x', 'y', 'theta', 'lookahead', 'speed']
    points = df[['x', 'y', 'lookahead']].values.tolist()
    return points[:max_records]

# def plot_data(points):
#     prev_lookahead = points[0][2]
#     change_points = []

#     for i in range(len(points) - 1):
#         x1, y1, lookahead1 = points[i]
#         x2, y2, lookahead2 = points[i + 1]
        
#         color = 'red' if lookahead1 == 2.5 else 'blue'
#         plt.plot([x1, x2], [y1, y2], color=color)
        
#         if lookahead1 == 2.5 and prev_lookahead != 2.5:
#             change_points.append((x1, y1, i))
        
#         prev_lookahead = lookahead1

#     # Add index numbers to the plot
#     for x, y, idx in change_points:
#         plt.text(x, y, str(idx), fontsize=9, ha='right', va='bottom')

#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.title(f'First {max_records} Line Segments Based on Lookahead')
#     plt.axis("equal")
#     plt.show()

def plot_data(points):
    prev_lookahead = points[0][2]
    change_points = []

    for i in range(len(points) - 1):
        x1, y1, lookahead1 = points[i]
        x2, y2, lookahead2 = points[i + 1]
        
        color = 'red' if lookahead1 == 2.5 else 'blue'
        plt.plot([x1, x2], [y1, y2], color=color)
        
        if (lookahead1 == 2.5 and prev_lookahead != 2.5) or (lookahead1 != 2.5 and prev_lookahead == 2.5):
            change_points.append((x1, y1, i))
        
        prev_lookahead = lookahead1

    # Add index numbers to the plot
    for x, y, idx in change_points:
        plt.text(x, y, str(idx), fontsize=9, ha='right', va='bottom')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(f'First {max_records} Line Segments Based on Lookahead')
    plt.axis("equal")
    plt.show()

file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
sheet_name = 'path_dubins'
data_points = read_data(file_path, sheet_name)
plot_data(data_points)

