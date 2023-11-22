#!/usr/bin/env python3
'''
The script is designed to:
- Read .csv file to get x, y points
- Craft a coverage map using the Boustrophedon approach

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage.py

'''


import pandas as pd

# Load the data
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'
data = pd.read_csv(file_path)

# Extract the X and Y coordinates
boundary_x = data['X'].values
boundary_y = data['Y'].values

# Create a shapely Polygon from the boundary points
from shapely.geometry import Polygon, LineString, Point
import numpy as np
boundary_polygon = Polygon([(x, y) for x, y in zip(boundary_x, boundary_y)])

# Calculate the bounding box of the polygon
min_x, min_y, max_x, max_y = boundary_polygon.bounds

# # Function to generate Boustrophedon coverage path
# def generate_boustrophedon_path(polygon, coverage_width):
#     """
#     Generate a Boustrophedon coverage path for a given polygon and coverage width.

#     :param polygon: Shapely Polygon representing the boundary of the area.
#     :param coverage_width: The width covered by the robot in each pass.
#     :return: List of points representing the coverage path.
#     """
#     # List to hold the coverage path points
#     coverage_path = []

#     # Calculate the number of lines needed, based on the coverage width
#     num_lines = int(np.ceil((max_y - min_y) / coverage_width))

#     # Generate lines and find intersections with the boundary
#     for i in range(num_lines):
#         # Calculate y coordinate of the line
#         y_coord = min_y + i * coverage_width

#         # Create a horizontal line at y_coord
#         line = LineString([(min_x, y_coord), (max_x, y_coord)])

#         # Find intersections with the polygon boundary
#         intersections = line.intersection(polygon.boundary)

#         # If there's no intersection or just a single point, continue
#         if intersections.is_empty or isinstance(intersections, Point):
#             continue

#         # Sort intersections from left to right
#         sorted_intersections = sorted(intersections, key=lambda point: point.x)

#         # Add intersections to the coverage path, alternating directions
#         if i % 2 == 0:
#             coverage_path.extend(sorted_intersections)
#         else:
#             coverage_path.extend(reversed(sorted_intersections))

#     return coverage_path

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
        if isinstance(intersections, MultiPoint):
            intersections = list(intersections)
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
boustrophedon_path = generate_boustrophedon_path(boundary_polygon, coverage_width)

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
