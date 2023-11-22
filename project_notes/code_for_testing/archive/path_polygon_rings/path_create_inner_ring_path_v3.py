#!/usr/bin/env python3  
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_step5_inner_rings_v2.py
'''
This script reads points from a CSV file containing (x, y) coordinates, calculates the inner rings an offset distance inside
 the polygon, and plots the original and new polygons:
'''

import csv
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import numpy as np

# def read_points_from_csv(file_path):
#     points = []
#     with open(file_path, 'r') as file:
#         reader = csv.reader(file)
#         next(reader)  # Skip the header row
#         for row in reader:
#             x, y = map(float, row)
#             points.append((x, y))
#     return points

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

def plot_polygons(original, inner_rings):
    plt.figure()
    original = np.array(original)
    plt.plot(original[:, 0], original[:, 1], 'b-', label='Original Polygon')
    plt.scatter(original[:, 0], original[:, 1], color='blue')

    colors = ['red', 'green', 'purple', 'orange', 'brown']
    for i, inner_ring in enumerate(inner_rings):
        inner_ring = np.array(inner_ring)
        plt.plot(inner_ring[:, 0], inner_ring[:, 1], color=colors[i], label=f'Inner Ring {i+1}')
        plt.scatter(inner_ring[:, 0], inner_ring[:, 1], color=colors[i])
    
    plt.axis('equal')
    plt.legend()
    plt.show()

# Save the entire path to a CSV file
with open('/mnt/data/continuous_path.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['x', 'y'])  # Write header
    for path in paths:
        for point in path:
            csvwriter.writerow(point)
print('Continuous path saved to continuous_path.csv')

def plot_innermost_ring(innermost_ring_points):
    # Convert the list of tuples to a list of x and y coordinates
    x_coords, y_coords = zip(*innermost_ring_points)
    
    # Create a scatter plot of the points
    plt.scatter(x_coords, y_coords, color='brown')
    
    # Annotate each point with its description
    for i, (x, y) in enumerate(innermost_ring_points, start=1):
        plt.annotate(f'Point {i}', (x, y), textcoords="offset points", xytext=(0, 10), ha='center')
    
    # Show the plot
    plt.axis('equal')
    plt.show()

# Save the entire path to a CSV file
with open('/mnt/data/continuous_path.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['x', 'y'])  # Write header
    for path in paths:
        for point in path:
            csvwriter.writerow(point)
print('Continuous path saved to continuous_path.csv')    

# File path to the CSV file
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'


# Read points from CSV
original_points = read_points_from_csv(file_path)

# Create Shapely Polygon from points
polygon = Polygon(original_points)

# Number of inner rings
num_inner_rings = 2

# Create inner rings and store them in a list
inner_rings = []
for i in range(1, num_inner_rings + 1):
    inner_ring = polygon.buffer(-i)  # Negative value for inner rings
    inner_points = list(inner_ring.exterior.coords)
    inner_rings.append(inner_points)


# Ensure the inner rings and original polygon are in clockwise order
def ensure_clockwise(polygon):
    if Polygon(polygon).area < 0:  # If area is negative, the polygon is clockwise
        return polygon[::-1]  # Reverse to make it clockwise
    return polygon

inner_ring_2 = ensure_clockwise(inner_rings[1])
inner_ring_1 = ensure_clockwise(inner_rings[0])
original_polygon_clockwise = ensure_clockwise(original_points)

# Create separate paths for each ring
paths = [inner_ring_2, inner_ring_1, original_polygon_clockwise]

# Print first and last coordinates of each ring
for i, path in enumerate(paths):
    print(f"Ring {i+1} starts at {path[0]} and ends at {path[-1]}")

# Plot each ring in a different color
plt.figure()
colors = ['red', 'green', 'blue']
labels = ['Inner Ring 2', 'Inner Ring 1', 'Original Polygon']
for i, path in enumerate(paths):
    plt.plot(*zip(*path), color=colors[i], label=labels[i])
    plt.scatter(*zip(*path), color=colors[i])
plt.axis('equal')
plt.legend()
plt.show()

# Save the entire path to a CSV file
with open('/mnt/data/continuous_path.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['x', 'y'])  # Write header
    for path in paths:
        for point in path:
            csvwriter.writerow(point)
print('Continuous path saved to continuous_path.csv')


innermost_ring_points = inner_rings[4]
print("Points of the Innermost (Brown) Ring:")
for point in innermost_ring_points:
    print(point)

#plot_innermost_ring(innermost_ring_points)