#!/usr/bin/env python3  
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_step5_inner_rings.py
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
            try:
                x, y = map(float, row)
                points.append(np.array([x, y]))
            except ValueError:
                print(f"Skipping row: {row}")
                continue
    return points

def calculate_inner_polygon(points, offset_distance):
    n = len(points)
    offset_points = []
    for i in range(n):
        p0, p1, p2 = points[i - 1], points[i], points[(i + 1) % n]
        v1 = (p1 - p0) / np.linalg.norm(p1 - p0)
        v2 = (p2 - p1) / np.linalg.norm(p2 - p1)
        normal = np.array([-v1[1], v1[0]])
        bisector = (v1 + v2) / np.linalg.norm(v1 + v2)
        direction = np.sign(np.dot(normal, bisector))
        offset = direction * normal * offset_distance
        offset_points.append(p1 + offset)
    return offset_points

def plot_polygons(original, offset):
    original = np.array(original)
    offset = np.array(offset)
    plt.plot(original[:, 0], original[:, 1], 'b-', label='Original Polygon')
    plt.plot(offset[:, 0], offset[:, 1], 'r-', label='Offset Polygon')
    plt.scatter(original[:, 0], original[:, 1], color='blue')
    plt.scatter(offset[:, 0], offset[:, 1], color='red')
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step2.csv'
    original_points = read_points_from_csv(file_path)
    offset_distance = -0.9  # ~ 35 inches inside
    polygon = Polygon(original_points)                      # Create Shapely Polygon from points
    inner_polygon = polygon.buffer(offset_distance)         # Create an inner polygon offset by 'offset_distance'
    inner_points = list(inner_polygon.exterior.coords)      # Get the exterior coordinates of the inner polygon
    plot_polygons(original_points, inner_points)            # Plot the original and inner polygons
