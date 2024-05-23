#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .csv file and outputs a series of concentric rings and stores them in a new sheet in the .ods file.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_create_rings5.py
'''


import pandas as pd
from shapely.geometry import Polygon, MultiPolygon
import matplotlib.pyplot as plt
import csv
import numpy as np

print("path_create_rings5.py starting....")
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
        colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown', 'black']
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
            for poly in inner_ring:
                paths.append(reorder_ring(list(poly.exterior.coords), start_point))
        else:
            paths.append(reorder_ring(list(inner_ring.exterior.coords), start_point))

    paths = [ensure_clockwise(path) for path in paths]
    write_paths_to_csv(paths, xy_file_name)
    
    original_ring = list(polygon.exterior.coords)
    paths.append(original_ring)
    
    plot_paths(paths)
    return paths

# Load the .ods file data
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'

with pd.ExcelFile(folder_path + file_path, engine='odf') as xls:
    sheet_names = xls.sheet_names
    sheet_data = {sheet: pd.read_excel(xls, sheet_name=sheet) for sheet in sheet_names}

# Print the contents of 'Obstacle 1' sheet
if 'Obstacle 1' in sheet_data:
    print("Contents of 'Obstacle 1' sheet before modification:")
    print(sheet_data['Obstacle 1'])
else:
    print("'Obstacle 1' sheet not found.")

# Filter the data for the concentric rings
filtered_data = sheet_data['SiteSurvey'][sheet_data['SiteSurvey']['Path Sequence'].between(1, 999)]
num_inner_rings = 3
path_size = 1.0
x_data = filtered_data['pose_x']
y_data = filtered_data['pose_y']
gps_data = list(zip(x_data, y_data))

start_point = (x_data.iloc[0], y_data.iloc[0]) if not x_data.empty else None
xy_file_name = folder_path + 'inner_ring_output_paths.csv'

if start_point is not None:
    inner_rings_paths = create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name)

    # Save inner rings to a separate sheet in the .ods file
    rows = []
    for path_index, path in enumerate(inner_rings_paths):
        for x, y in path:
            rows.append({'Path_Index': path_index, 'X': x, 'Y': y})
    inner_rings_df = pd.DataFrame(rows)

    sheet_data['RawInnerRings'] = inner_rings_df

    with pd.ExcelWriter(folder_path + file_path, engine='odf') as writer:
        for sheet_name, df in sheet_data.items():
            df.to_excel(writer, sheet_name=sheet_name, index=False)

    print("Inner rings processing completed.")
else:
    print("No valid data points found. Inner rings not created.")

# Verify the contents of 'Obstacle 1' sheet after modification
with pd.ExcelFile(folder_path + file_path, engine='odf') as xls:
    sheet_names_after = xls.sheet_names
    sheet_data_after = {sheet: pd.read_excel(xls, sheet_name=sheet) for sheet in sheet_names_after}

if 'Obstacle 1' in sheet_data_after:
    print("Contents of 'Obstacle 1' sheet after modification:")
    print(sheet_data_after['Obstacle 1'])
else:
    print("'Obstacle 1' sheet not found.")
