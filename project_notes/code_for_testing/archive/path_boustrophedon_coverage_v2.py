#!/usr/bin/env python3
'''
The script is designed to:
- Read .csv file to get x, y points
- Craft a coverage map using the Boustrophedon approach

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage_v2.py

'''

import numpy as np
import pandas as pd
from shapely.geometry import Polygon, LineString, Point, MultiPoint
import matplotlib.pyplot as plt

# Load the data
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'
data = pd.read_csv(file_path)

# Extract the X and Y coordinates
boundary_x = data['X'].values
boundary_y = data['Y'].values

# Create a shapely Polygon from the boundary points
boundary_polygon = Polygon([(x, y) for x, y in zip(boundary_x, boundary_y)])

# Function to generate Boustrophedon coverage path
def generate_boustrophedon_path_fixed(polygon, coverage_width):
    coverage_path = []  # List to hold the coverage path points
    min_x, min_y, max_x, max_y = polygon.bounds  # Calculate the bounding box of the polygon
    num_lines = int(np.ceil((max_y - min_y) / coverage_width))  # Calculate the number of lines needed

    for i in range(num_lines):
        y_coord = min_y + i * coverage_width  # Calculate y coordinate of the line
        line = LineString([(min_x, y_coord), (max_x, y_coord)])  # Create a horizontal line at y_coord
        intersections = line.intersection(polygon.boundary)  # Find intersections with the polygon boundary

        if intersections.is_empty:  # If there's no intersection, continue
            continue
        
        # Convert MultiPoint to a list of points if necessary
        # if isinstance(intersections, MultiPoint):
        #     intersections = list(intersections)
        # elif isinstance(intersections, Point):
        #     intersections = [intersections]

        if isinstance(intersections, MultiPoint):
            intersections = [point for point in intersections.geoms]
        elif isinstance(intersections, Point):
            intersections = [intersections]        

        sorted_intersections = sorted(intersections, key=lambda point: point.x)  # Sort intersections

        # Add intersections to the coverage path, alternating directions
        if i % 2 == 0:
            coverage_path.extend(sorted_intersections)
        else:
            coverage_path.extend(reversed(sorted_intersections))

    return coverage_path

# Generate the Boustrophedon coverage path
coverage_width = 0.9  # 0.9 meters coverage width
boustrophedon_path = generate_boustrophedon_path_fixed(boundary_polygon, coverage_width)

# Extract x and y coordinates from the path
path_x, path_y = zip(*[(point.x, point.y) for point in boustrophedon_path])

# Plot the coverage path
plt.figure(figsize=(10, 10))
plt.plot(boundary_x, boundary_y, 'b-', label='Boundary')
plt.plot(path_x, path_y, 'g-', label='Coverage Path')
plt.scatter(boundary_x, boundary_y, c='red', label='Vertices')
plt.title('Boustrophedon Coverage Path')
plt.xlabel('X Coordinate (m)')
plt.ylabel('Y Coordinate (m)')
plt.axis('equal')  # Ensure equal scaling for x and y axes
plt.legend()
plt.grid(True)
plt.show()
