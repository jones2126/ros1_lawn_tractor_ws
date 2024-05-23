#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and modify the path to avoid the obstacle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_find_intersects2.py

'''
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Define the function to calculate the shortest path around the circle
def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points):
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

# Function to identify the indices where the Path_Index changes
def get_path_index_changes(raw_inner_rings):
    path_index_changes = []
    current_index = raw_inner_rings.iloc[0, 0]
    for i in range(1, len(raw_inner_rings)):
        if raw_inner_rings.iloc[i, 0] != current_index:
            path_index_changes.append(i)
            current_index = raw_inner_rings.iloc[i, 0]
    return path_index_changes

# Main function to process intersections and update the revised path
def find_intersections_with_circle(ods_file_path): 
    # Load data from the .ods file
    raw_inner_rings = pd.read_excel(ods_file_path, sheet_name='RawInnerRings', engine='odf')
    obstacle_segments = pd.read_excel(ods_file_path, sheet_name='Obstacle 1', engine='odf')

    circle_segments = [((row['Start X'], row['Start Y']), (row['End X'], row['End Y'])) for _, row in obstacle_segments.iterrows()]
    circle_center = (np.mean([seg[0][0] for seg in circle_segments]), np.mean([seg[0][1] for seg in circle_segments]))
    radius = np.mean([np.sqrt((seg[0][0] - circle_center[0])**2 + (seg[0][1] - circle_center[1])**2) for seg in circle_segments])

    intersection_points = []
    intersecting_segments = []
    intersecting_index = []
    cir_seg_intersects = []
    current_path_index = None

    for i in range(len(raw_inner_rings) - 1):
        seg1_start = (raw_inner_rings.iloc[i, 1], raw_inner_rings.iloc[i, 2])
        seg1_end = (raw_inner_rings.iloc[i + 1, 1], raw_inner_rings.iloc[i + 1, 2])
        path_index = raw_inner_rings.iloc[i, 0]
        cir_seg_ndx = 1

        if path_index != current_path_index:
            print(f"Looking for intersects in path {path_index}")
            current_path_index = path_index

        for this_segment in circle_segments:
            seg2_start, seg2_end = this_segment
            intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
            if intersects_sw:
                if intersects_pt:  # Ensure intersects_pt is not None
                    if intersects_pt != (0, 0):  # Avoid adding the placeholder (0, 0) as an intersection point
                        if intersects_pt not in intersection_points:  # Avoid adding duplicate intersection points
                            print(f"Intersection found between path segment {seg1_start}-{seg1_end} and circle segment {seg2_start}-{seg2_end}")
                            intersection_points.append(intersects_pt)
                            intersecting_segments.append((seg1_start, seg1_end))
                            intersecting_index.append(i)
                            cir_seg_intersects.append(cir_seg_ndx)
            cir_seg_ndx += 1

    print("Intersection points:", intersection_points)
    print("Rows that intersected with the circle:", intersecting_index)
    print("Circle segments with intersects:", cir_seg_intersects)

    revised_path = pd.DataFrame()

    if intersecting_index:
        current_path_index = raw_inner_rings.iloc[0, 0]
        temp_path = raw_inner_rings.iloc[0:intersecting_index[0]].copy()
        temp_path['Source'] = 'path_segment_0'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

        qty_of_intersects = len(intersecting_index)

        # Initialize a counter for unique source values
        source_counter = 0

        # Get the list of indices where the Path_Index changes
        path_index_changes = get_path_index_changes(raw_inner_rings)

        for i in range(0, qty_of_intersects - 1, 2):
            point1 = intersection_points[i]
            point2 = intersection_points[i + 1]
            path_index = raw_inner_rings.iloc[intersecting_index[i], 0]  # Use the same Path_Index for the new segment

            circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, 20)
            circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
            circle_arc_df['Path_Index'] = path_index
            circle_arc_df['Source'] = f'circle_arc_df_{source_counter}'
            revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

            source_counter += 1

            if i + 2 < qty_of_intersects:
                temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:intersecting_index[i + 2]].copy()
            else:
                temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:].copy()

            # Determine the correct Path_Index based on the x and y coordinates
            temp_path_index = temp_path['Path_Index'].iloc[0]
            for change_idx in path_index_changes:
                if temp_path['X'].iloc[0] >= raw_inner_rings.iloc[change_idx, 1] and temp_path['Y'].iloc[0] >= raw_inner_rings.iloc[change_idx, 2]:
                    temp_path_index = raw_inner_rings.iloc[change_idx, 0]
                    break

            temp_path['Path_Index'] = temp_path_index
            temp_path['Source'] = f'path_segment_{source_counter}'
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

            source_counter += 1

        if i + 1 < qty_of_intersects:
            temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:].copy()
            temp_path_index = temp_path['Path_Index'].iloc[0]
            for change_idx in path_index_changes:
                if temp_path['X'].iloc[0] >= raw_inner_rings.iloc[change_idx, 1] and temp_path['Y'].iloc[0] >= raw_inner_rings.iloc[change_idx, 2]:
                    temp_path_index = raw_inner_rings.iloc[change_idx, 0]
                    break

            temp_path['Path_Index'] = temp_path_index
            temp_path['Source'] = f'end_path_{source_counter}'
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

    # Read existing sheets
    with pd.ExcelFile(ods_file_path, engine='odf') as xls:
        sheet_names = xls.sheet_names
        sheet_data = {sheet: pd.read_excel(xls, sheet_name=sheet) for sheet in sheet_names}

    # Add the new sheet data
    sheet_data['NewRingPath'] = revised_path

    # Write all sheets back to the .ods file
    with pd.ExcelWriter(ods_file_path, engine='odf') as writer:
        for sheet_name, df in sheet_data.items():
            df.to_excel(writer, sheet_name=sheet_name, index=False)

    return intersection_points, intersecting_segments, intersecting_index, cir_seg_intersects

# Function to plot the new ring path
def plot_new_ring_path(ods_file_path):
    df = pd.read_excel(ods_file_path, sheet_name='NewRingPath', engine='odf')
    plt.figure(figsize=(10, 6))
    for path_index in df['Path_Index'].unique():
        path_data = df[df['Path_Index'] == path_index]
        plt.scatter(path_data['X'], path_data['Y'], label=f'Path Index {path_index}', s=10)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('New Ring Path')
    plt.legend()
    plt.grid(True)
    plt.show()

# Function to plot the obstacle circle
def plot_obstacle_circle(ods_file_path):
    obstacle_segments = pd.read_excel(ods_file_path, sheet_name='Obstacle 1', engine='odf')
    plt.figure(figsize=(10, 6))
    for _, row in obstacle_segments.iterrows():
        plt.plot([row['Start X'], row['End X']], [row['Start Y'], row['End Y']], 'b-')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Obstacle Circle')
    plt.grid(True)
    plt.show()

# Define the function to determine if segments intersect
def do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end):
    def cross_product(a, b):
        return a[0] * b[1] - a[1] * b[0]

    def subtract_vectors(a, b):
        return (a[0] - b[0], a[1] - b[1])

    def is_point_on_segment(seg_start, seg_end, point):
        seg = subtract_vectors(seg_end, seg_start)
        pt_seg = subtract_vectors(point, seg_start)
        cross_prod = cross_product(seg, pt_seg)
        if abs(cross_prod) > np.finfo(float).eps:
            return False  # Point is not on the line created by the segment
        dot_prod = np.dot(seg, pt_seg)
        if dot_prod < 0 or dot_prod > np.dot(seg, seg):
            return False  # Point is outside the segment
        return True

    # Calculate vectors
    r = subtract_vectors(seg1_end, seg1_start)
    s = subtract_vectors(seg2_end, seg2_start)
    qp = subtract_vectors(seg2_start, seg1_start)

    # Calculate cross products and check if line segments are parallel
    r_cross_s = cross_product(r, s)
    qp_cross_r = cross_product(qp, r)

    # If r_cross_s is zero, line segments are parallel or collinear
    if abs(r_cross_s) < np.finfo(float).eps:
        # If qp_cross_r is also zero, line segments are collinear
        if abs(qp_cross_r) < np.finfo(float).eps:
            # Check if the segments overlap
            if is_point_on_segment(seg1_start, seg1_end, seg2_start) or is_point_on_segment(seg1_start, seg1_end, seg2_end) or \
               is_point_on_segment(seg2_start, seg2_end, seg1_start) or is_point_on_segment(seg2_start, seg2_end, seg1_end):
                return True, (0, 0)
        return False, None

    # Calculate the scalar multiples t and u for the intersection point
    t = cross_product(qp, s) / r_cross_s
    u = cross_product(qp, r) / r_cross_s

    # Check if the scalar multiples are within the range [0, 1] for both line segments
    if 0 <= t <= 1 and 0 <= u <= 1:
        # Calculate the intersection point
        intersection_x = seg1_start[0] + t * (seg1_end[0] - seg1_start[0])
        intersection_y = seg1_start[1] + t * (seg1_end[1] - seg1_start[1])
        return True, (intersection_x, intersection_y)
    return False, None

# main function
#
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
find_intersections_with_circle(ods_file_path)
plot_new_ring_path(ods_file_path)
plot_obstacle_circle(ods_file_path)
plt.show()
