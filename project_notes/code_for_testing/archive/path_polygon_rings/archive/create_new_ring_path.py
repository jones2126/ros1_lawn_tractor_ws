#!/usr/bin/env python

'''
Script to read intersection points, calculate the shortest distance around obstacles, and create the 'NewRingPath'.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/create_new_ring_path.py
'''

print("create_new_ring_path.py starting....")

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance

# Function to calculate the shortest path around the obstacle
def calculate_shortest_path(intersection_points, circle_center, circle_radius):
    # Placeholder for your path calculation logic
    # This function should calculate the shortest path around the obstacle
    # using the intersection points, circle center, and radius.
    # Replace this placeholder with your actual path calculation logic.
    return intersection_points  # Replace this with the actual new ring path

# Function to create the new ring path
def create_new_ring_path(xlsx_file_path):
    df_intersection_points = pd.read_excel(xlsx_file_path, sheet_name='IntersectionPoints', engine='openpyxl')
    print(f"Read IntersectionPoints sheet: {df_intersection_points}")

    # Group by obstacle sheet to handle multiple obstacles
    grouped = df_intersection_points.groupby('Obstacle_Sheet')

    revised_path = pd.DataFrame(columns=['X', 'Y', 'Path_Index'])

    for obstacle_sheet, group in grouped:
        print(f"Processing {obstacle_sheet}")
        circle_center = (group.iloc[0]['Segment_Start_X'], group.iloc[0]['Segment_Start_Y'])  # Update this to get the actual circle center
        circle_radius = group.iloc[0]['Radius']  # Update this to get the actual radius

        intersection_points = group[['Intersection_X', 'Intersection_Y']].values

        new_path_segment = calculate_shortest_path(intersection_points, circle_center, circle_radius)

        revised_path = pd.concat([revised_path, pd.DataFrame(new_path_segment, columns=['X', 'Y'])], ignore_index=True)

    # Add the new sheet 'NewRingPath' to the .xlsx file, removing it first if it already exists
    with pd.ExcelWriter(xlsx_file_path, engine='openpyxl', mode='a') as writer:
        workbook = writer.book
        if 'NewRingPath' in workbook.sheetnames:
            del workbook['NewRingPath']
        revised_path.to_excel(writer, sheet_name='NewRingPath', index=False)

    print("NewRingPath created and saved to 'NewRingPath' sheet.")

# Main Function
xlsx_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
create_new_ring_path(xlsx_file_path)
