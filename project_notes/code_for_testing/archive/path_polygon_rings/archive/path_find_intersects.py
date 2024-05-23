#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and modify the path to avoid the obstacle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_find_intersects.py
'''
#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and modify the path to avoid the obstacle.
'''

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Function to check if two segments intersect
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

    r = subtract_vectors(seg1_end, seg1_start)
    s = subtract_vectors(seg2_end, seg2_start)
    qp = subtract_vectors(seg2_start, seg1_start)
    r_cross_s = cross_product(r, s)
    qp_cross_r = cross_product(qp, r)

    if abs(r_cross_s) < np.finfo(float).eps:
        if abs(qp_cross_r) < np.finfo(float).eps:
            if is_point_on_segment(seg1_start, seg1_end, seg2_start) or is_point_on_segment(seg1_start, seg1_end, seg2_end) or \
               is_point_on_segment(seg2_start, seg2_end, seg1_start) or is_point_on_segment(seg2_start, seg2_end, seg1_end):
                return True, None
        return False, None

    t = cross_product(qp, s) / r_cross_s
    u = cross_product(qp, r) / r_cross_s

    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection_x = seg1_start[0] + t * (seg1_end[0] - seg1_start[0])
        intersection_y = seg1_start[1] + t * (seg1_end[1] - seg1_start[1])
        return True, (intersection_x, intersection_y)
    return False, None

# Function to plot the circle and intersections
def plot_circle_and_intersections(center, radius, circle_segments, intersection_points, intersecting_segments):
    plt.figure()

    for i, seg in enumerate(circle_segments):
        if i == 0:
            plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-', label='Circle Segments')
        else:
            plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-')

    if intersection_points:
        offset = 0.1
        for idx, point in enumerate(intersection_points):
            plt.plot(point[0], point[1], 'ro')
            plt.text(point[0] + offset, point[1] + offset, str(idx + 1), color='black', fontsize=10)

    if intersecting_segments:
        plt.plot([intersecting_segments[0][0][0], intersecting_segments[0][1][0]], 
                 [intersecting_segments[0][0][1], intersecting_segments[0][1][1]], 'y-', label='Intersecting Segments')
        for seg in intersecting_segments[1:]:
            plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'y-')

    plt.title('Circle and Intersections')
    plt.legend()
    plt.axis('equal')
    plt.show()

# Function to calculate the shortest path around the circle
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

# Function to read and plot the 'NewRingPath' data
def plot_new_ring_path(ods_file_path):
    new_ring_path = pd.read_excel(ods_file_path, sheet_name='NewRingPath', engine='odf')
    print(f"Rows in 'NewRingPath': {len(new_ring_path)}")
    plt.figure(figsize=(10, 6))
    plt.autoscale(enable=True, axis='both', tight=None)
    for path_index in new_ring_path['Path_Index'].unique():
        path_data = new_ring_path[new_ring_path['Path_Index'] == path_index]
        plt.scatter(path_data['X'], path_data['Y'], alpha=0.6, label=f'Path Index {path_index}')
    plt.title('New Ring Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

# Function to plot the obstacle circle from 'Obstacle 1' sheet
def plot_obstacle_circle(ods_file_path):
    obstacle_segments = pd.read_excel(ods_file_path, sheet_name='Obstacle 1', engine='odf')
    circle_segments = [((row['Start X'], row['Start Y']), (row['End X'], row['End Y'])) for _, row in obstacle_segments.iterrows()]

    for i, seg in enumerate(circle_segments):
        plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'g-', label='Obstacle Circle' if i == 0 else "")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

# Main function to find intersections
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

    plot_circle_and_intersections(None, None, circle_segments, intersection_points, intersecting_segments)

    revised_path = pd.DataFrame()

    if intersecting_index:
        temp_path = raw_inner_rings.iloc[0:intersecting_index[0]].copy()
        revised_path = pd.DataFrame()
        temp_path['Source'] = 'temp_path_beg'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

        qty_of_intersects = len(intersecting_index)
        new_path_index = 0

        for i in range(0, qty_of_intersects - 1, 2):
            point1 = intersection_points[i]
            if i + 1 < qty_of_intersects:
                point2 = intersection_points[i + 1]
                circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, 20)
                circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
                circle_arc_df['Path_Index'] = new_path_index
                circle_arc_df['Source'] = 'circle_arc_df'
                revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)
                new_path_index += 1

                if i + 2 < qty_of_intersects:
                    temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:intersecting_index[i + 2]].copy()
                else:
                    temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:].copy()
                temp_path['Path_Index'] = new_path_index
                temp_path['Source'] = 'temp_path'
                revised_path = pd.concat([revised_path, temp_path], ignore_index=True)
                new_path_index += 1

        if i + 1 < qty_of_intersects:
            point2 = intersection_points[i + 1]
            temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:].copy()
            temp_path['Path_Index'] = new_path_index
            temp_path['Source'] = 'end_path'
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

# Usage example
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
find_intersections_with_circle(ods_file_path)
plot_new_ring_path(ods_file_path)
plot_obstacle_circle(ods_file_path)
plt.show()
