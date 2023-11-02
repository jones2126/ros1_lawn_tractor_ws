
from shapely.geometry import Point
import numpy as np

def simplify_offset_polygon(offset_points, distance_threshold):
    distances = [Point(offset_points[i]).distance(Point(offset_points[i+1])) for i in range(len(offset_points)-1)]
    clusters = []
    current_cluster = []
    for i, d in enumerate(distances):
        if d < distance_threshold:
            if not current_cluster:
                current_cluster.append(offset_points[i])
            current_cluster.append(offset_points[i+1])
        else:
            if current_cluster:
                clusters.append(current_cluster)
                current_cluster = []
    if current_cluster:
        clusters.append(current_cluster)
    middle_points = [cluster[len(cluster)//2] for cluster in clusters]
    new_polygon_points = [p for p in offset_points if not any(p in cluster for cluster in clusters)]
    new_polygon_points.extend(middle_points)
    new_polygon_points = sorted(new_polygon_points, key=lambda p: (np.arctan2(p[1], p[0]), p))
    return new_polygon_points


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

def read_points_from_csv(file_path):
    points = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            x, y = map(float, row)
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

# File path to the CSV file
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step2.csv'

# Read points from CSV
original_points = read_points_from_csv(file_path)

# Create Shapely Polygon from points
polygon = Polygon(original_points)

# Number of inner rings
num_inner_rings = 5

# Distance threshold for simplification
distance_threshold = 0.1

# Create inner rings and store them in a list
inner_rings = []
for i in range(1, num_inner_rings + 1):
    inner_ring = polygon.buffer(-i)  # Negative value for inner rings
    inner_points = list(inner_ring.exterior.coords)
    simplified_points = simplify_offset_polygon(inner_points, distance_threshold)
    inner_rings.append(simplified_points)

# Plot the original polygon and the inner rings
plot_polygons(original_points, inner_rings)

innermost_ring_points = inner_rings[4]
print("Points of the Innermost (Brown) Ring:")
for point in innermost_ring_points:
    print(point)

#plot_innermost_ring(innermost_ring_points)