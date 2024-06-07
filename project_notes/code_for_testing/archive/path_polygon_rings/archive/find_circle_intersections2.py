#!/usr/bin/env python

'''
Script to find intersections between circles defined in 'Obstcl_list' and the line segments in 'RawInnerRings'.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/find_circle_intersections.py
'''

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Function to check if a point is in the circle
def is_point_in_circle(point, center, radius):
    return np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2) <= radius

# Function to find the intersection points of a line segment with a circle
def find_intersection_points(seg_start, seg_end, center, radius):
    """
    Find the intersection points of a line segment with a circle.
    
    Parameters:
    seg_start (array-like): The starting point of the line segment (x, y).
    seg_end (array-like): The ending point of the line segment (x, y).
    center (array-like): The center of the circle (x, y).
    radius (float): The radius of the circle.
    
    Returns:
    list: A list of intersection points. Each point is represented as an array-like (x, y).
    """
    d = np.subtract(seg_end, seg_start)      # Vector from start to end of the segment
    f = np.subtract(seg_start, center)       # Vector from center of the circle to the start of the segment
    
    # Coefficients for the quadratic equation
    a = np.dot(d, d)
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
def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points=20):
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

# Function to find intersections and save to Excel
def find_intersections_with_circle(xlsx_file_path):
    # Read data from Excel file
    df_obstcl_list = pd.read_excel(xlsx_file_path, sheet_name='Obstcl_list')
    df_raw_inner_rings = pd.read_excel(xlsx_file_path, sheet_name='RawInnerRings')

    # Debug: Print the column names to verify they are correct
    print("RawInnerRings columns:", df_raw_inner_rings.columns)
    print("Obstcl_list columns:", df_obstcl_list.columns)

    all_intersection_data = []
    for _, row in df_raw_inner_rings.iterrows():
        seg_start = (row['Start_X'], row['Start_Y'])
        seg_end = (row['End_X'], row['End_Y'])
        
        for _, obstacle in df_obstcl_list.iterrows():
            circle_center = (obstacle['X'], obstacle['Y'])
            circle_radius = obstacle['Radius']
            
            intersection_points = find_intersection_points(seg_start, seg_end, circle_center, circle_radius)
            for point in intersection_points:
                all_intersection_data.append({
                    'Segment_Start_X': seg_start[0],
                    'Segment_Start_Y': seg_start[1],
                    'Segment_End_X': seg_end[0],
                    'Segment_End_Y': seg_end[1],
                    'Circle_Center_X': circle_center[0],
                    'Circle_Center_Y': circle_center[1],
                    'Circle_Radius': circle_radius,
                    'Intersection_X': point[0],
                    'Intersection_Y': point[1]
                })

    if not all_intersection_data:
        print("No intersections found.")
        return None

    all_intersection_data = pd.DataFrame(all_intersection_data)
    print(f"Found {len(all_intersection_data)} intersection points.")

    # Calculate shortest path around the circle for the first intersection
    circle_center = (all_intersection_data.iloc[0]['Circle_Center_X'], all_intersection_data.iloc[0]['Circle_Center_Y'])
    radius = all_intersection_data.iloc[0]['Circle_Radius']
    start_point = (all_intersection_data.iloc[0]['Intersection_X'], all_intersection_data.iloc[0]['Intersection_Y'])
    end_point = (all_intersection_data.iloc[1]['Intersection_X'], all_intersection_data.iloc[1]['Intersection_Y'])  # Assuming at least 2 intersections

    arc_path = calculate_shortest_path(circle_center, radius, start_point, end_point)

    arc_path_df = pd.DataFrame(arc_path, columns=['Arc_X', 'Arc_Y'])

    # Add the intersection data and arc path to a new sheet 'IntersectionPoints'
    with pd.ExcelWriter(xlsx_file_path, engine='openpyxl', mode='a') as writer:
        workbook = writer.book
        if 'IntersectionPoints' in workbook.sheetnames:
            del workbook['IntersectionPoints']
        all_intersection_data.to_excel(writer, sheet_name='IntersectionPoints', index=False)
        arc_path_df.to_excel(writer, sheet_name='ArcPath', index=False)

    # Plot the intersection points and the line segments
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

    # Plot the arc path
    arc_path = np.array(arc_path)
    plt.plot(arc_path[:, 0], arc_path[:, 1], 'b--', label='Shortest Path Around Circle')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.title('Path with Intersections and Shortest Path Around Circle')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Main Function
xlsx_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
find_intersections_with_circle(xlsx_file_path)
