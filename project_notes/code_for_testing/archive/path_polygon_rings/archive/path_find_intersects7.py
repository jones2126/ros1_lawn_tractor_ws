#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and store the intersection points in a new sheet 'IntersectionPoints'.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_find_intersects7.py
'''

print("path_find_intersects7.py starting....")

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from openpyxl import load_workbook
import sys

# Function to find intersections with circle
def find_intersections_with_circle(xlsx_file_path, df_raw_inner_rings):
    obstacle_sheets = [sheet for sheet in pd.ExcelFile(xlsx_file_path).sheet_names if sheet.startswith('Obstacle')]
    print(f"Found obstacle sheets: {obstacle_sheets}")

    all_intersection_data = pd.DataFrame()

    for obstacle_sheet in obstacle_sheets:
        df_obstacle_circle = pd.read_excel(xlsx_file_path, sheet_name=obstacle_sheet, engine='openpyxl')
        print(f"Read {obstacle_sheet} sheet: {df_obstacle_circle}")

        # Ensure required columns are present
        required_columns = ['X', 'Y', 'Radius']
        for col in required_columns:
            if col not in df_obstacle_circle.columns:
                print(f"Error: Column '{col}' not found in {obstacle_sheet} sheet. Available columns: {df_obstacle_circle.columns}")
                return

        circle_center = (df_obstacle_circle.iloc[0]['X'], df_obstacle_circle.iloc[0]['Y'])
        circle_radius = df_obstacle_circle.iloc[0]['Radius']

        print(f"Circle center: {circle_center}, Circle radius: {circle_radius}")

        def is_point_in_circle(point, center, radius):
            return np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2) <= radius

        def find_intersection_point(seg_start, seg_end, center, radius):
            d = np.subtract(seg_end, seg_start)
            f = np.subtract(seg_start, center)
            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - radius**2
            discriminant = b**2 - 4*a*c

            if discriminant < 0:
                return None
            else:
                discriminant = np.sqrt(discriminant)
                t1 = (-b - discriminant) / (2*a)
                t2 = (-b + discriminant) / (2*a)
                intersection_points = []
                if 0 <= t1 <= 1:
                    intersection_points.append(seg_start + t1 * d)
                if 0 <= t2 <= 1:
                    intersection_points.append(seg_start + t2 * d)
                return intersection_points if intersection_points else None

        intersection_points = []
        intersecting_segments = []
        intersecting_index = []

        for i in range(len(df_raw_inner_rings) - 1):
            seg_start = np.array([df_raw_inner_rings.at[i, 'X'], df_raw_inner_rings.at[i, 'Y']])
            seg_end = np.array([df_raw_inner_rings.at[i + 1, 'X'], df_raw_inner_rings.at[i + 1, 'Y']])
            
            if is_point_in_circle(seg_start, circle_center, circle_radius) or is_point_in_circle(seg_end, circle_center, circle_radius):
                print(f"Segment {i} intersects with circle")
                intersection_points_list = find_intersection_point(seg_start, seg_end, circle_center, circle_radius)
                if intersection_points_list is not None:
                    for intersection_point in intersection_points_list:
                        print(f"Intersection found at {intersection_point}")
                        intersection_points.append(intersection_point)
                        intersecting_segments.append((seg_start, seg_end))
                        intersecting_index.append(i)

        # Create a DataFrame to store intersection points with obstacle sheet name
        intersection_data = pd.DataFrame({
            'Intersection_X': [point[0] for point in intersection_points],
            'Intersection_Y': [point[1] for point in intersection_points],
            'Segment_Start_X': [seg[0][0] for seg in intersecting_segments],
            'Segment_Start_Y': [seg[0][1] for seg in intersecting_segments],
            'Segment_End_X': [seg[1][0] for seg in intersecting_segments],
            'Segment_End_Y': [seg[1][1] for seg in intersecting_segments],
            'Path_Index': intersecting_index,
            'Obstacle_Sheet': [obstacle_sheet] * len(intersection_points)
        })

        all_intersection_data = pd.concat([all_intersection_data, intersection_data], ignore_index=True)

    # Check the length of the intersection data
    if len(all_intersection_data) > 5000:
        print(f"Error: Intersection data has too many records ({len(all_intersection_data)}). Stopping program.")
        return None

    # Add the intersection data to the .xlsx file, removing it first if it already exists
    with pd.ExcelWriter(xlsx_file_path, engine='openpyxl', mode='a') as writer:
        workbook = writer.book
        if 'IntersectionPoints' in workbook.sheetnames:
            del workbook['IntersectionPoints']
        all_intersection_data.to_excel(writer, sheet_name='IntersectionPoints', index=False)

    # Plot the intersection points, line segments, and label them
    plt.figure(figsize=(10, 6))

    for i, point in enumerate(intersection_points):
        segment = intersecting_segments[i]
        plt.plot([segment[0][0], segment[1][0]], [segment[0][1], segment[1][1]], 'r-', label=f'Segment {intersecting_index[i]}')
        plt.scatter(point[0], point[1], color='blue', s=50, zorder=5)
        plt.text(point[0], point[1], f'{intersecting_index[i]}', fontsize=12, ha='right')

    for obstacle_sheet in obstacle_sheets:
        df_obstacle_circle = pd.read_excel(xlsx_file_path, sheet_name=obstacle_sheet, engine='openpyxl')
        circle_center = (df_obstacle_circle.iloc[0]['X'], df_obstacle_circle.iloc[0]['Y'])
        circle_radius = df_obstacle_circle.iloc[0]['Radius']
        plt.scatter(circle_center[0], circle_center[1], color='green', s=100, label='Obstacle Center')
        circle = plt.Circle(circle_center, circle_radius, color='green', fill=False, label=obstacle_sheet)
        plt.gca().add_patch(circle)
        
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Path with Intersections')
    plt.grid(True)
    plt.axis('equal')

    plt.show()

# Main Function
xlsx_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'

# Adding debug to print initial path data
df_raw_inner_rings = pd.read_excel(xlsx_file_path, sheet_name='RawInnerRings', engine='openpyxl')
print(f"Initial data from RawInnerRings:\n{df_raw_inner_rings}")

find_intersections_with_circle(xlsx_file_path, df_raw_inner_rings)
