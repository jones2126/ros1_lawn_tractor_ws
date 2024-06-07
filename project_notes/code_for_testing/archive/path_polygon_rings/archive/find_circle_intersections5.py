#!/usr/bin/env python

'''
Script to find intersections between circles defined in 'Obstcl_list' and the line segments in 'RawInnerRings'.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/find_circle_intersections5.py
'''

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import openpyxl

# Function to check if a point is in the circle
def is_point_in_circle(point, center, radius):
    return np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2) <= radius

# Function to find the intersection points of a line segment with a circle
def find_intersection_points(seg_start, seg_end, center, radius, epsilon=1e-10):
    """
    Find the intersection points of a line segment with a circle.
    
    Parameters:
    seg_start (array-like): The starting point of the line segment (x, y).
    seg_end (array-like): The ending point of the line segment (x, y).
    center (array-like): The center of the circle (x, y).
    radius (float): The radius of the circle.
    epsilon (float): A small value to avoid division by zero.
    
    Returns:
    list: A list of intersection points. Each point is represented as an array-like (x, y).
    """
    d = np.subtract(seg_end, seg_start)      # Vector from start to end of the segment
    f = np.subtract(seg_start, center)       # Vector from center of the circle to the start of the segment
    
    # Coefficients for the quadratic equation
    a = np.dot(d, d) + epsilon
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - radius**2
    
    discriminant = b**2 - 4*a*c             # Discriminant of the quadratic equation

    # If discriminant is negative, there are no real roots, hence no intersection
    if discriminant < 0:
        return []
    else:
        # Compute the two points of intersection
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)
        
        # Check if the intersection points are within the segment
        intersection_points = []
        if 0 <= t1 <= 1:
            intersection_points.append(np.add(seg_start, t1 * d))
        if 0 <= t2 <= 1:
            intersection_points.append(np.add(seg_start, t2 * d))
        
        return intersection_points

# Function to calculate the shortest path around a circle
def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points=12):
    def adjust_angle(angle):
        while angle < 0:
            angle += 2 * np.pi
        while angle > 2 * np.pi:
            angle -= 2 * np.pi
        return angle

    def angle_between_points(center, point):
        angle = np.arctan2(point[1] - center[1], point[0] - center[0])
        return angle

    start_angle = adjust_angle(angle_between_points(circle_center, start_point))
    end_angle = adjust_angle(angle_between_points(circle_center, end_point))
    angular_difference = end_angle - start_angle
    if angular_difference > np.pi:
        angular_difference -= 2 * np.pi
    elif angular_difference < -np.pi:
        angular_difference += 2 * np.pi

    arc_path = []
    for i in range(num_points + 1):
        angle = start_angle + angular_difference * i / num_points
        x = circle_center[0] + radius * np.cos(angle)
        y = circle_center[1] + radius * np.sin(angle)
        arc_path.append((x, y))
    return arc_path

# Function to find intersections with circles and save to Excel
def find_intersections_with_circle(xlsx_file_path):
    df_raw_inner_rings = pd.read_excel(xlsx_file_path, sheet_name='RawInnerRings', engine='openpyxl')
    print(f"Initial data from RawInnerRings:\n{df_raw_inner_rings}")

    df_obstcl_list = pd.read_excel(xlsx_file_path, sheet_name='Obstcl_list', engine='openpyxl')
    print(f"Initial data from Obstcl_list:\n{df_obstcl_list}")

    all_intersection_data = pd.DataFrame()

    for idx, obstacle in df_obstcl_list.iterrows():
        circle_center = (obstacle['X'], obstacle['Y'])
        circle_radius = obstacle['Radius']
        reference = obstacle['Reference']
        print(f"Processing obstacle {idx + 1}: center={circle_center}, radius={circle_radius}")

        intersection_points = []
        intersecting_segments = []
        intersecting_index = []

        for i in range(len(df_raw_inner_rings) - 1):
            seg_start = np.array([df_raw_inner_rings.at[i, 'X'], df_raw_inner_rings.at[i, 'Y']])
            seg_end = np.array([df_raw_inner_rings.at[i + 1, 'X'], df_raw_inner_rings.at[i + 1, 'Y']])
            
            intersections = find_intersection_points(seg_start, seg_end, circle_center, circle_radius)
            for intersection in intersections:
                intersection_points.append(intersection)
                intersecting_segments.append((seg_start, seg_end))
                intersecting_index.append(i)

        # Create a DataFrame to store intersection points with obstacle index
        intersection_data = pd.DataFrame({
            'Intersection_X': [point[0] for point in intersection_points],
            'Intersection_Y': [point[1] for point in intersection_points],
            'Segment_Start_X': [seg[0][0] for seg in intersecting_segments],
            'Segment_Start_Y': [seg[0][1] for seg in intersecting_segments],
            'Segment_End_X': [seg[1][0] for seg in intersecting_segments],
            'Segment_End_Y': [seg[1][1] for seg in intersecting_segments],
            'Path_Index': intersecting_index,
            'Obstacle_Index': [idx] * len(intersection_points),
            'Circle_Center_X': [circle_center[0]] * len(intersection_points),
            'Circle_Center_Y': [circle_center[1]] * len(intersection_points),
            'Circle_Radius': [circle_radius] * len(intersection_points),
            'Reference': [reference] * len(intersection_points)
        })
        print(f"idx: {[idx]} and  len(intersection_points): {len(intersection_points)}")

        all_intersection_data = pd.concat([all_intersection_data, intersection_data], ignore_index=True)

    # Check the length of the intersection data
    if len(all_intersection_data) > 5000:
        print(f"Error: Intersection data has too many records ({len(all_intersection_data)}). Stopping program.")
        return None

    # Add the intersection data to a new sheet 'IntersectionPoints'
    with pd.ExcelWriter(xlsx_file_path, engine='openpyxl', mode='a') as writer:
        workbook = writer.book
        if 'IntersectionPoints' in workbook.sheetnames:
            del workbook['IntersectionPoints']
        all_intersection_data.to_excel(writer, sheet_name='IntersectionPoints', index=False)

    return all_intersection_data

# Function to calculate and store shortest path
def calculate_and_store_shortest_path(xlsx_file_path, all_intersection_data):
    arc_path_data = []

    # Group the intersection data by 'Obstacle_Index' and 'Path_Index'
    grouped = all_intersection_data.groupby(['Obstacle_Index', 'Path_Index'])
    
    for (obstacle_idx, path_idx), group in grouped:
        if len(group) > 1:
            circle_center = (group.iloc[0]['Circle_Center_X'], group.iloc[0]['Circle_Center_Y'])
            radius = group.iloc[0]['Circle_Radius']
            reference = group.iloc[0]['Reference']
            
            for i in range(len(group) - 1):
                start_point = (group.iloc[i]['Intersection_X'], group.iloc[i]['Intersection_Y'])
                end_point = (group.iloc[i + 1]['Intersection_X'], group.iloc[i + 1]['Intersection_Y'])
                
                arc_path = calculate_shortest_path(circle_center, radius, start_point, end_point, num_points=12)
                for arc_point in arc_path:
                    arc_path_data.append({
                        'Arc_X': arc_point[0],
                        'Arc_Y': arc_point[1],
                        'Path_Index': path_idx,
                        'Reference': reference
                    })
            print(f"Processed Obstacle Index: {obstacle_idx}, Path Index: {path_idx}")

    arc_path_df = pd.DataFrame(arc_path_data)
    print(f"arc_path_df:\n{arc_path_df}")

    # Load the workbook and remove the existing 'ArcPath' sheet if it exists
    with pd.ExcelWriter(xlsx_file_path, engine='openpyxl', mode='a') as writer:
        workbook = writer.book
        if 'ArcPath' in workbook.sheetnames:
            del workbook['ArcPath']
        arc_path_df.to_excel(writer, sheet_name='ArcPath', index=False)

# Function to plot intersection points, segments, and circles
def plot_intersections(xlsx_file_path, all_intersection_data):
    df_obstcl_list = pd.read_excel(xlsx_file_path, sheet_name='Obstcl_list', engine='openpyxl')

    plt.figure(figsize=(10, 6))
    for i, point in enumerate(all_intersection_data[['Intersection_X', 'Intersection_Y']].values):
        segment = all_intersection_data.iloc[i][['Segment_Start_X', 'Segment_Start_Y', 'Segment_End_X', 'Segment_End_Y']].values.reshape(2, 2)
        plt.plot([segment[0][0], segment[1][0]], [segment[0][1], segment[1][1]], 'r-', label=f'Segment {all_intersection_data.iloc[i]["Path_Index"]}' if i == 0 else "")
        plt.scatter(point[0], point[1], color='blue', s=50, zorder=5)
        plt.text(point[0], point[1], f'{all_intersection_data.iloc[i]["Path_Index"]}', fontsize=12, ha='right')

    # Plot the circles
    for idx, obstacle in df_obstcl_list.iterrows():
        circle_center = (obstacle['X'], obstacle['Y'])
        circle_radius = obstacle['Radius']
        plt.scatter(circle_center[0], circle_center[1], color='green', s=100, label=f'Obstacle {idx + 1} Center' if idx == 0 else "")
        circle = plt.Circle(circle_center, circle_radius, color='green', fill=False, label=f'Obstacle {idx + 1}')
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
all_intersection_data = find_intersections_with_circle(xlsx_file_path)
calculate_and_store_shortest_path(xlsx_file_path, all_intersection_data)
plot_intersections(xlsx_file_path, all_intersection_data)
