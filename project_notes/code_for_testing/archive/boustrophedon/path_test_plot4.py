#!/usr/bin/env python

'''
Read the data from the specified sheet, and create Dubins path u-turns.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_plot4.py
'''
import pandas as pd
import matplotlib.pyplot as plt
import math

def calculate_angle(x1, y1, x2, y2):
    '''Calculates the directional angle with respect to the positive X-axis.'''
    angle = math.atan2(y2 - y1, x2 - x1)
    if angle < 0:
        angle += 2 * math.pi
    return angle

print("Starting the process...")

# Define the input file path and sheet name
input_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
input_sheet_name = 'coverage_path_refrmttd'

# Read the Excel sheet
print(f"Reading sheet '{input_sheet_name}' from the Excel file...")
df = pd.read_excel(input_file_path, sheet_name=input_sheet_name, header=None)

# Extract the first 20 x and y points
x = df.iloc[:20, 0].tolist()
y = df.iloc[:20, 1].tolist()

print(f"Extracted the first {len(x)} x and y points.")

# Calculate angles
angles = []
for i in range(len(x) - 1):
    angle = calculate_angle(x[i], y[i], x[i+1], y[i+1])
    angles.append(angle)

# Create the plot
print("Creating the plot...")
plt.figure(figsize=(12, 10))
plt.plot(x, y, 'b-o')  # Blue line with circle markers

if len(x) > 0:
    plt.scatter(x[0], y[0], color='green', s=100, label='Start')  # Green marker for start point
    if len(x) > 1:
        plt.scatter(x[-1], y[-1], color='red', s=100, label='End')  # Red marker for end point

# Customize the plot
plt.title(f'First {len(x)} Points of Coverage Path with Angles')
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.grid(True)
plt.legend()

# Add point labels and angles
for i, (xi, yi) in enumerate(zip(x, y)):
    plt.annotate(f'P{i}', (xi, yi), xytext=(5, 5), textcoords='offset points')
    if i < len(angles):
        plt.annotate(f'{angles[i]:.2f}', (xi, yi), xytext=(5, -15), textcoords='offset points', color='red')

print("Plot created. Displaying the plot...")

# Show the plot
plt.tight_layout()
plt.show()

print("Process completed.")