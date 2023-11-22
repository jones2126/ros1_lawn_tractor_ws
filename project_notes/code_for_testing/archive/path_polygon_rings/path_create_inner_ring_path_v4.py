#!/usr/bin/env python3  
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_create_inner_ring_path_v4.py
'''
This script reads points from a CSV file containing (x, y) coordinates, calculates the inner rings an offset distance inside
 the polygon, and plots the original and new polygons:
'''

import csv
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np
import math

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

def read_points_from_csv(file_path):
    points = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            # Columns J and K correspond to indices 9 and 10
            x, y = map(float, [row[9], row[10]])
            points.append((x, y))
    return points

# Ensure the inner rings and original polygon are in clockwise order
def ensure_clockwise(polygon):
    if Polygon(polygon).area < 0:  # If area is negative, the polygon is clockwise
        return polygon[::-1]  # Reverse to make it clockwise
    return polygon    

# Function to reorder a ring so that it starts near a specified point
def reorder_ring(ring, start_point):
    # Find the point in the ring closest to the start_point
    closest_index = min(range(len(ring)), key=lambda i: (ring[i][0] - start_point[0])**2 + (ring[i][1] - start_point[1])**2)
    # Reorder the ring to start from the closest point
    return ring[closest_index:] + ring[:closest_index]

# Define the function to calculate the ROS angle
def calculate_ROS_angle(x1, y1, x2, y2):
    '''
    Calculates the directional angle with respect to the positive X-axis. This is in line with the ROS REP 103 standard, where an angle 
    of 0 radians corresponds to movement directly along the positive X-axis and approximately 1.57 radians corresponds to movement 
    directly along the positive Y-axis.
    '''    
    angle = math.atan2(y2 - y1, x2 - x1)
    if angle < 0:  # Ensures the angle is between 0 and 2*pi
        angle += 2 * math.pi
    return angle


# File path to the CSV file
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'
output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path.csv'
original_points = read_points_from_csv(file_path)       # Read points from CSV
polygon = Polygon(original_points)                      # Create Shapely Polygon from points

# Create inner rings and store them in a list
num_inner_rings = 2
path_size = 0.9
inner_rings = []
for i in range(1, num_inner_rings + 1):
    inner_ring = polygon.buffer(-path_size * i)  # Set gap for path.  Needs to be negative for inner rings
    inner_points = list(inner_ring.exterior.coords)
    inner_rings.append(inner_points)

start_point = (-7.6, 2.4)
inner_ring_2 = reorder_ring(ensure_clockwise(inner_rings[1]), start_point)
inner_ring_1 = reorder_ring(ensure_clockwise(inner_rings[0]), start_point)
original_polygon_clockwise = reorder_ring(ensure_clockwise(original_points)[::-1], start_point)


paths = [inner_ring_2, inner_ring_1, original_polygon_clockwise]        # Create separate paths for each ring

# Print first and last coordinates of each ring
for i, path in enumerate(paths):
    print(f"Ring {i+1} starts at {path[0]} and ends at {path[-1]}")



# # Plot each ring in a different color
# plt.figure()
# colors = ['red', 'green', 'blue']
# labels = ['Inner Ring 2', 'Inner Ring 1', 'Original Polygon']
# for i, path in enumerate(paths):
#     plt.plot(*zip(*path), color=colors[i], label=labels[i])
#     plt.scatter(*zip(*path), color=colors[i])

# # # Drawing the circle
# # circle_center = (17.6, -9.5)
# # circle_radius = 5.4 / 2  # Since 5.4 is the diameter
# # Circle parameters
# center_x, center_y = 17.6, -9.5
# radius = 5.4 / 2

# # circle = plt.Circle(circle_center, circle_radius, color='purple', fill=False)
# # plt.gca().add_patch(circle)


# # Generating points at 30 degree intervals
# angles = np.arange(0, 360, 30)  # Degrees from 0 to 330
# points_x = center_x + radius * np.cos(np.radians(angles))
# points_y = center_y + radius * np.sin(np.radians(angles))

# # Plotting
# plt.figure(figsize=(8, 8))
# circle = plt.Circle((center_x, center_y), radius, color='blue', fill=False)
# plt.gca().add_patch(circle)
# plt.scatter(points_x, points_y, color='red')

# # Annotating points
# for x, y in zip(points_x, points_y):
#     plt.annotate(f'({x:.2f}, {y:.2f})', (x, y), textcoords="offset points", xytext=(0,10), ha='center')

# plt.xlim(center_x - radius - 1, center_x + radius + 1)
# plt.ylim(center_y - radius - 1, center_y + radius + 1)
# plt.gca().set_aspect('equal', adjustable='box')



# plt.title(f'Script: {script_name}\nData source: {file_path}')
# plt.figtext(0.5, 0.01, f'Data saved to: {output_file_path}', ha='center', fontsize=8, color='gray')

# plt.axis('equal')
# plt.legend()
# plt.show()

plt.figure(figsize=(8, 8))

# Plot the paths
colors = ['red', 'green', 'blue']
labels = ['Inner Ring 2', 'Inner Ring 1', 'Original Polygon']
for i, path in enumerate(paths):
    plt.plot(*zip(*path), color=colors[i], label=labels[i])
    plt.scatter(*zip(*path), color=colors[i])

# Draw the circle
center_x, center_y = 17.6, -9.5
radius = 5.4 / 2
circle = plt.Circle((center_x, center_y), radius, color='blue', fill=False)
plt.gca().add_patch(circle)

# Generate and plot points at 30 degree intervals
angles = np.arange(0, 360, 30)
points_x = center_x + radius * np.cos(np.radians(angles))
points_y = center_y + radius * np.sin(np.radians(angles))
plt.scatter(points_x, points_y, color='red')

# Annotate points
for x, y in zip(points_x, points_y):
    plt.annotate(f'({x:.2f}, {y:.2f})', (x, y), textcoords="offset points", xytext=(0,10), ha='center')

plt.xlim(center_x - radius - 5, center_x + radius + 5)
plt.ylim(center_y - radius - 5, center_y + radius + 5)
plt.gca().set_aspect('equal', adjustable='box')
plt.title(f'Script: {script_name}\nData source: {file_path}')
plt.figtext(0.5, 0.01, f'Data saved to: {output_file_path}', ha='center', fontsize=8, color='gray')

plt.axis('equal')
plt.legend()
plt.show()


# # Calculate angle and append data with the angle for all points except the last one
data_with_angles = []

# Iterate through each ring
for ring in paths:
    # Iterate through each point in the ring
    for i in range(len(ring) - 1):
        x1, y1 = ring[i]
        x2, y2 = ring[i + 1]
        angle = calculate_ROS_angle(x1, y1, x2, y2)
        data_with_angles.append((x1, y1, angle))

# The last point in the last ring should have the same angle as the second to last point
if data_with_angles:
    data_with_angles.append((ring[-1][0], ring[-1][1], data_with_angles[-1][2]))

# Save the entire path to a CSV file
with open(output_file_path, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['x', 'y', 'angle'])  # Write header
    for point in data_with_angles:
        csvwriter.writerow(point)  # Write each point directly            