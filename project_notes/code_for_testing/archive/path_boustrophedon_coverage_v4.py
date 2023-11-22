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
from shapely.affinity import rotate
import matplotlib.pyplot as plt
import warnings

# Treat all instances of a certain warning as exceptions
warnings.filterwarnings('error', category=RuntimeWarning)

# Load the data
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'
data = pd.read_csv(file_path)

# Extract the X and Y coordinates
boundary_x = data['X'].values
boundary_y = data['Y'].values

# Create a shapely Polygon from the boundary points
boundary_polygon = Polygon([(x, y) for x, y in zip(boundary_x, boundary_y)])

# # First, let's define a function that rotates a point around the origin (0,0) by a given angle.
# def rotate_point(point, angle):
#     """
#     Rotate a point counterclockwise by a given angle around the origin.

#     The angle should be given in radians.
#     """
#     ox, oy = 0, 0
#     px, py = point

#     qx = ox + np.cos(angle) * (px - ox) - np.sin(angle) * (py - oy)
#     qy = oy + np.sin(angle) * (px - ox) + np.cos(angle) * (py - oy)
#     return qx, qy

# Function to generate Boustrophedon coverage path
def generate_boustrophedon_path(polygon, coverage_width, slope_angle):
    coverage_path = []  # List to hold the coverage path points
    min_x, min_y, max_x, max_y = polygon.bounds  # Calculate the bounding box of the polygon
    
    # Calculate the number of lines needed, accounting for the slope by extending the bounding box
    diag = np.hypot(max_x - min_x, max_y - min_y)
    num_lines = int(np.ceil(diag / coverage_width / np.cos(slope_angle)))  # Adjust for slope
    
    for i in range(num_lines):
        # Calculate y coordinate of the line, then create a line at y_coord
        y_coord = min_y + i * coverage_width / np.cos(slope_angle)  # Adjust for slope
        line = LineString([(min_x, y_coord), (max_x, y_coord)])
        
        # Rotate the line around the center of the bounding box by the slope_angle
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        rotated_line = rotate(line, slope_angle, origin=(center_x, center_y), use_radians=True)
        
        # Find intersections with the polygon boundary
        #intersections = rotated_line.intersection(polygon.boundary)

        try:
            intersections = rotated_line.intersection(polygon.boundary)
        except RuntimeWarning as e:
            print(f"A RuntimeWarning occurred during intersection calculation: {e}")
            print(f"Line: {rotated_line}")
            #print(f"Polygon Boundary: {polygon.boundary}")
            # Optionally, handle the warning or continue
            continue  # Skip this iteration

        
        if intersections.is_empty:  # If there's no intersection, continue
            continue
        
        # Convert MultiPoint to a list of points if necessary
        if isinstance(intersections, MultiPoint):
            intersections = [point for point in intersections.geoms]
        elif isinstance(intersections, Point):
            intersections = [intersections]
        
        # Sort intersections along the rotated line direction
        sorted_intersections = sorted(intersections, key=lambda point: point.distance(Point(center_x, center_y)))
        
        # Add intersections to the coverage path, alternating directions
        if i % 2 == 0:
            coverage_path.extend(sorted_intersections)
        else:
            coverage_path.extend(reversed(sorted_intersections))

    return coverage_path

def plot_boustrophedon_path(polygon, path, slope_angle):
    x, y = polygon.exterior.xy  # Get the x and y coordinates of the polygon's boundary
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'k')  # Plot the polygon boundary in black
    
    # Plot the Boustrophedon path
    xs = [point.x for point in path]
    ys = [point.y for point in path]
    plt.plot(xs, ys, 'b')  # Plot the path in blue
    
    # Plot a green circle at the start and a red square at the end of the path
    if path:
        plt.plot(xs[0], ys[0], 'go', markersize=10)  # Start point
        plt.plot(xs[-1], ys[-1], 'rs', markersize=10)  # End point
    
    plt.axis('equal')
    plt.grid(True)
    plt.title('Boustrophedon Path with Slope Angle: {:.2f} radians'.format(slope_angle))
    plt.show()


# Generate the Boustrophedon coverage path
coverage_width = 0.9  # 0.9 meters coverage width
slope_angle = 0.75
boustrophedon_path = generate_boustrophedon_path(boundary_polygon, coverage_width, slope_angle)
plot_boustrophedon_path(boundary_polygon, boustrophedon_path, slope_angle)

# # Extract x and y coordinates from the path
# path_x, path_y = zip(*[(point.x, point.y) for point in boustrophedon_path])

# # Plot the coverage path
# plt.figure(figsize=(10, 10))
# plt.plot(boundary_x, boundary_y, 'b-', label='Boundary')
# plt.plot(path_x, path_y, 'g-', label='Coverage Path')
# plt.scatter(boundary_x, boundary_y, c='red', label='Vertices')
# plt.title('Boustrophedon Coverage Path')
# plt.xlabel('X Coordinate (m)')
# plt.ylabel('Y Coordinate (m)')
# plt.axis('equal')  # Ensure equal scaling for x and y axes
# plt.legend()
# plt.grid(True)
# plt.show()
