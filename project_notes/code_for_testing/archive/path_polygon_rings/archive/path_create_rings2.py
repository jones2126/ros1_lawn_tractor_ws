#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .csv file and outputs a series of concentric rings.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_create_rings2.py


'''

import pandas as pd
from shapely.geometry import Polygon, MultiPolygon
import matplotlib.pyplot as plt
import csv

# Define the function to create inner rings
def create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name):
    from shapely.geometry import Polygon, MultiPolygon
    import matplotlib.pyplot as plt
    import csv

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

    # Use gps_data directly since it's already a list of tuples
    polygon = Polygon(gps_data)  # Create polygon directly from gps_data
    paths = []
    for i in range(num_inner_rings, 0, -1):
        inner_ring = polygon.buffer(-path_size * i)
        if isinstance(inner_ring, MultiPolygon):
            for polygon in inner_ring.geoms:
                inner_points = list(polygon.exterior.coords)
        else:
            inner_points = list(inner_ring.exterior.coords)
        reordered_ring = reorder_ring(ensure_clockwise(inner_points), start_point)
        paths.append(reordered_ring)
    paths.append(reorder_ring(ensure_clockwise(gps_data), start_point))  # Use gps_data directly
    write_paths_to_csv(paths, xy_file_name)
    plot_paths(paths)
    return paths

# Load the .ods file data
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
print("Loading data from .ods file.")
data = pd.read_excel(folder_path + file_path, engine='odf')
# Filter data based on 'Path Sequence' having values between 1 and 999
filtered_data = data[data['Path Sequence'].between(1, 999)]
x_data = filtered_data['pose_x']
y_data = filtered_data['pose_y']
gps_data = list(zip(x_data, y_data))  # List of tuples

# Set parameters for the inner ring creation
num_inner_rings = 3
# for path size, 42 inches×0.0254 meters/inch=1.0668 meters
path_size = 1.0
start_point = (x_data.iloc[0], y_data.iloc[0]) if not x_data.empty else None
xy_file_name = folder_path + 'inner_ring_output_paths.csv'

# Execute the function
if start_point is not None:
    print("Creating inner rings.")
    inner_rings_paths = create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name)
    print("Inner rings processing completed.")
else:
    print("No valid data points found. Inner rings not created.")
