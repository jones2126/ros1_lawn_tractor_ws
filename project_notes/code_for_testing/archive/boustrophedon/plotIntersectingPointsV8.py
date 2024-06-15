#!/usr/bin/env python

'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/plotIntersectingPointsV8.py
'''

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import matplotlib.patheffects as patheffects

script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

"""
This program calculates and visualizes perpendicular intercept points between consecutive line segments. It is designed to work 
with a set of line segments, each defined by two endpoints, and it computes the points where perpendicular lines from the 
endpoints of one segment intersect the next segment.

Functionality:

Data Input:
The program reads line segment data from a CSV file. Each row in this file represents a line segment, specified by four 
values: x1, y1, x2, y2. These values represent the coordinates of the starting (x1, y1) and ending (x2, y2) points of each segment.

Calculation of Perpendicular Intercepts:
For each line segment, the program computes two perpendicular intercept points:
Upper Intersect: A perpendicular line from the endpoint with the higher y value (y2) to the next line segment.
Lower Intersect: A perpendicular line from the endpoint with the lower y value (y1) to the next line segment.
These intercept points are calculated only if they lie on the next line segment. If no valid intercept is found, the program 
skips to the next pair of segments.

Data Update and Output:
The original DataFrame is updated with new columns: upper_intercept_x, upper_intercept_y, lower_intercept_x, and 
lower_intercept_y, representing the x and y coordinates of the upper and lower intercept points, respectively.

The updated DataFrame is then saved to a new CSV file.

Visualization:
The program generates a plot showing all the line segments along with the calculated perpendicular intercept points.
Line segments are displayed in blue, while intercept points are marked in red.

Error Handling:
The program includes logic to handle cases where a valid perpendicular intercept does not exist (i.e., the perpendicular 
line from a segment endpoint does not intersect the next segment).

"""

# Load the Excel file
xlsx_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
sheet_name = 'boustrphdn_segmnts'
df = pd.read_excel(xlsx_file_path, sheet_name=sheet_name)

# Updated script to attempt a secondary intercept calculation if the primary attempt fails

def perpendicular_intercept(x1, y1, x2, y2, next_x1, next_y1, next_x2, next_y2, is_upper):
    # Calculate slope and intercept of the original line
    slope = (y2 - y1) / (x2 - x1)
    intercept = y1 - slope * x1

    # Calculate the perpendicular slope
    perp_slope = -1 / slope

    # Use the upper or lower point as reference for the intercept point
    x_ref, y_ref = (x2, y2) if is_upper else (x1, y1)

    # Calculate the intercept of the perpendicular line
    perp_intercept = y_ref - perp_slope * x_ref

    # Calculate the intercept point with the next line segment
    next_slope = (next_y2 - next_y1) / (next_x2 - next_x1)
    next_intercept = next_y1 - next_slope * next_x1

    # Calculate the intersection point
    x_intercept = (perp_intercept - next_intercept) / (next_slope - perp_slope)
    y_intercept = perp_slope * x_intercept + perp_intercept

    # Check if the intersection point lies on the next line segment
    if min(next_x1, next_x2) <= x_intercept <= max(next_x1, next_x2) and min(next_y1, next_y2) <= y_intercept <= max(next_y1, next_y2):
        return x_intercept, y_intercept
    else:
        return None, None

# Function to calculate the perpendicular intercept point with a secondary check
def perpendicular_intercept_with_secondary_check(x1, y1, x2, y2, next_x1, next_y1, next_x2, next_y2, is_upper):
    primary_intercept = perpendicular_intercept(x1, y1, x2, y2, next_x1, next_y1, next_x2, next_y2, is_upper)
    if primary_intercept != (None, None):
        return primary_intercept
    else:
        # Attempt to find intercept from 'n+1' to 'n'
        return perpendicular_intercept(next_x1, next_y1, next_x2, next_y2, x1, y1, x2, y2, is_upper)

# Calculate the intercept points with secondary check
upper_intercepts = []
lower_intercepts = []
for index, row in df.iterrows():
    if index < len(df) - 1:
        next_row = df.iloc[index + 1]
        upper_intercept = perpendicular_intercept_with_secondary_check(row['x1'], row['y1'], row['x2'], row['y2'],
                                                                       next_row['x1'], next_row['y1'], next_row['x2'], next_row['y2'], True)
        lower_intercept = perpendicular_intercept_with_secondary_check(row['x1'], row['y1'], row['x2'], row['y2'],
                                                                       next_row['x1'], next_row['y1'], next_row['x2'], next_row['y2'], False)
        upper_intercepts.append(upper_intercept)
        lower_intercepts.append(lower_intercept)

# Update the DataFrame with the new intercept points
df['upper_intercept_x'] = pd.Series([x for x, y in upper_intercepts])
df['upper_intercept_y'] = pd.Series([y for x, y in upper_intercepts])
df['lower_intercept_x'] = pd.Series([x for x, y in lower_intercepts])
df['lower_intercept_y'] = pd.Series([y for x, y in lower_intercepts])

# Trim the edges that will be taken care of with paths around the edges
first_rows_to_remove = 1
end_rows_to_remove = 2

if first_rows_to_remove > 0:
    df = df.iloc[first_rows_to_remove:].reset_index(drop=True)

if end_rows_to_remove > 0:
    end_rows_to_remove = end_rows_to_remove * -1
    df = df[:end_rows_to_remove].reset_index(drop=True)

# Plotting the updated line segments and intercept points with index labels
plt.figure(figsize=(10, 6))
for index, row in df.iterrows():
    # Plot line segments
    plt.plot([row['x1'], row['x2']], [row['y1'], row['y2']], 'b-')
    # Position the text in the center of the line segment
    text_x = (row['x1'] + row['x2']) / 2
    text_y = (row['y1'] + row['y2']) / 2
    plt.text(text_x, text_y, str(index + first_rows_to_remove + 1), fontsize=8, verticalalignment='center',
             horizontalalignment='center', color='white', path_effects=[
             patheffects.withStroke(linewidth=2, foreground="black")])
    # Plot upper intercept points
    if not np.isnan(row['upper_intercept_x']):
        plt.plot(row['upper_intercept_x'], row['upper_intercept_y'], 'ro')
    # Plot lower intercept points
    if not np.isnan(row['lower_intercept_x']):
        plt.plot(row['lower_intercept_x'], row['lower_intercept_y'], 'ro')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Line Segments and Perpendicular Intercept Points with Secondary Check after Trimming')
plt.grid(True)
plt.show()

# Save the updated DataFrame to a new sheet in the Excel file
output_sheet_name = 'intercepts'
with pd.ExcelWriter(xlsx_file_path, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
    df.to_excel(writer, sheet_name=output_sheet_name, index=False)
print(f"saving the intercept points to file: {xlsx_file_path} in sheet: {output_sheet_name}")
print("Copy these points to the 'helper' spreadsheet")
