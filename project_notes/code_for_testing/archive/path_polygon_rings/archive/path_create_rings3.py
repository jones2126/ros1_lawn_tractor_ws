#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .csv file and outputs a series of concentric rings and processes intersections with a circle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_create_rings3.py

'''

import pandas as pd
from shapely.geometry import Polygon, MultiPolygon
import matplotlib.pyplot as plt
import csv
import numpy as np
from matplotlib.patches import Circle

# Define the function to create inner rings
def create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name):
    def reorder_ring(ring, start_point):
        closest_index = min(range(len(ring)), key=lambda i: (ring[i][0] - start_point[0])**2 + (ring[i][1] - start_point[1])**2)
        return ring[closest_index:] + ring[:closest_index]

    def ensure_clockwise(polygon):
        if Polygon(polygon).area < 0:
            return polygon[::-1]
        return polygon

    def write_paths_to_csv(paths, file_name):
        with open(file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Path_Index', 'X', 'Y'])
            for index, path in enumerate(paths):
                for x, y in path:
                    writer.writerow([index, x, y])

    def plot_paths(paths):
        plt.figure(figsize=(10, 8))
        colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown']
        for path, color in zip(paths, colors):
            x_coords, y_coords = zip(*path)
            plt.plot(x_coords, y_coords, marker='o', color=color)
        plt.title('Plot of Paths')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    polygon = Polygon(gps_data)
    paths = []
    for i in range(num_inner_rings, 0, -1):
        inner_ring = polygon.buffer(-path_size * i)
        if isinstance(inner_ring, MultiPolygon):
            for poly in inner_ring.geoms:
                inner_points = list(poly.exterior.coords)
        else:
            inner_points = list(inner_ring.exterior.coords)
        reordered_ring = reorder_ring(ensure_clockwise(inner_points), start_point)
        paths.append(reordered_ring)
    paths.append(reorder_ring(ensure_clockwise(gps_data), start_point))
    write_paths_to_csv(paths, xy_file_name)
    plot_paths(paths)
    return paths

# Define the function to find intersections with a circle
def find_intersections_with_circle(xy_file_name, circle_center, radius, num_circle_points, output_file_path): 
    def calculate_circle_arc(circle_center, radius, num_points):
        segments = []
        angles = np.linspace(0, 2 * np.pi, num_points + 1)
        for i in range(num_points):
            start_angle = angles[i]
            end_angle = angles[i + 1]
            start_point = (circle_center[0] + radius * np.cos(start_angle), circle_center[1] + radius * np.sin(start_angle))
            end_point = (circle_center[0] + radius * np.cos(end_angle), circle_center[1] + radius * np.sin(end_angle))
            segments.append((start_point, end_point))
        return segments

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
                return False
            dot_prod = np.dot(seg, pt_seg)
            if dot_prod < 0 or dot_prod > np.dot(seg, seg):
                return False
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
                    return True, (0, 0)
            return False, None

        t = cross_product(qp, s) / r_cross_s
        u = cross_product(qp, r) / r_cross_s

        if 0 <= t <= 1 and 0 <= u <= 1:
            intersection_x = seg1_start[0] + t * (seg1_end[0] - seg1_start[0])
            intersection_y = seg1_start[1] + t * (seg1_end[1] - seg1_start[1])
            return True, (intersection_x, intersection_y)
        return False, None

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

    def plot_circle_and_intersections(center, radius, circle_segments, intersection_points, intersecting_segments, script_name, csv_file_path):
        plt.figure()
        circle = Circle(center, radius, color='green', fill=False)
        plt.gca().add_patch(circle)
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
        plt.title(f'Script: {script_name}\nData source: {csv_file_path}')
        plt.legend()
        plt.axis('equal')
        plt.show()

    def plot_xy_coordinates(csv_file):
        data = pd.read_csv(csv_file)
        plt.figure(figsize=(10, 6))
        plt.autoscale(enable=True, axis='both', tight=None)
        plt.scatter(data['X'], data['Y'], alpha=0.6)
        plt.title('X and Y Coordinates')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')
        plt.grid(True)
        plt.show()        

    xy_file_df = pd.read_csv(xy_file_name)
    print(f"Number of rows in DataFrame: {len(xy_file_df)}")
    circle_segments = calculate_circle_arc(circle_center, radius, num_circle_points)
    print(f"Number of rows in circle_segments: {len(circle_segments)}")

    intersection_points = []
    intersecting_segments = []
    intersecting_index = []
    cir_seg_intersects = []
    current_path_index = None

    for i in range(len(xy_file_df) - 1):
        seg1_start = (xy_file_df.iloc[i, 1], xy_file_df.iloc[i, 2])
        seg1_end = (xy_file_df.iloc[i + 1, 1], xy_file_df.iloc[i + 1, 2])
        cir_seg_ndx = 1
        path_index = xy_file_df.iloc[i, 0]

        if path_index != current_path_index:
            print(f"Looking for intersects in path {path_index}")
            current_path_index = path_index

        for this_segment in circle_segments:
            seg2_start, seg2_end = this_segment
            intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
            if intersects_sw:
                intersection_points.append(intersects_pt)
                intersecting_segments.append((seg1_start, seg1_end))
                intersecting_index.append(i)
                cir_seg_intersects.append(cir_seg_ndx)
            cir_seg_ndx += 1

    print("Intersection points:", intersection_points)
    print("xy_file_name rows that intersected with the circle:", intersecting_index)
    print("Circle segments with intersects:", cir_seg_intersects)
    plot_circle_and_intersections(circle_center, radius, circle_segments, intersection_points, intersecting_segments, "path_create_rings2.py", xy_file_name)

    revised_path = pd.DataFrame()

    if intersecting_index:
        temp_path = xy_file_df.iloc[0:intersecting_index[0]].copy()
        revised_path = pd.DataFrame()
        temp_path['Source'] = 'temp_path_beg'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

        qty_of_intersects = len(intersecting_index) - 2
        print("qty_of_intersects: ", qty_of_intersects)
        print("looping sequence for obstacle path")
        for i, path_index in zip(range(0, qty_of_intersects, 2), range(0, 4)):
            point1 = intersection_points[i]
            point2 = intersection_points[i + 1]
            print(i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)
            circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_circle_points)
            circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
            circle_arc_df['Path_Index'] = path_index
            circle_arc_df['Source'] = 'circle_arc_df'
            revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

            temp_path = xy_file_df.iloc[intersecting_index[i+1] + 1:intersecting_index[i+2]].copy()
            temp_path['Path_Index'] = path_index
            temp_path['Source'] = 'temp_path'
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)
        print("End of loop:", i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)

        print("Handling the end segment")
        path_index = path_index + 1
        i = i + 2
        point1 = intersection_points[i]
        point2 = intersection_points[i + 1]
        print(i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)
        circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_circle_points)
        circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
        circle_arc_df['Path_Index'] = path_index
        circle_arc_df['Source'] = 'circle_arc_df_end'
        revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

        temp_path = xy_file_df.iloc[intersecting_index[i+1] + 1:].copy()
        temp_path['Path_Index'] = path_index
        temp_path['Source'] = 'end_path'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

    revised_path.to_csv(output_file_path, index=False)
    if intersecting_index:
        plot_xy_coordinates(output_file_path)
    return intersection_points, intersecting_segments, intersecting_index, cir_seg_intersects


# Load the .ods file data
print("Loading data from .ods file.")
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
data = pd.read_excel(folder_path + file_path, engine='odf')
filtered_data = data[data['Path Sequence'].between(1, 999)]
x_data = filtered_data['pose_x']
y_data = filtered_data['pose_y']
gps_data = list(zip(x_data, y_data))

num_inner_rings = 3
path_size = 1.0
start_point = (x_data.iloc[0], y_data.iloc[0]) if not x_data.empty else None
xy_file_name = folder_path + 'inner_ring_output_paths.csv'

# Execute the function to create inner rings
if start_point is not None:
    print("Creating inner rings.")
    inner_rings_paths = create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name)
    print("Inner rings processing completed.")
else:
    print("No valid data points found. Inner rings not created.")

# Define parameters for the intersection function
circle_center = (-10.5, -33.2)
radius = 4.8
num_circle_points = 21
xy_file_adjusted_for_obstacles = folder_path + 'Site_01_ring_adjusted_for_obstacles.csv'

# Execute the function to find intersections with a circle
if start_point is not None:
    intersections, segments, indices, circle_segments = find_intersections_with_circle(xy_file_name, circle_center, radius, num_circle_points, xy_file_adjusted_for_obstacles)
    print("Intersection processing completed.")
else:
    print("No valid data points found. Intersection processing not executed.")
