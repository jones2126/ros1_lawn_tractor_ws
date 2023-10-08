#!/usr/bin/env python
'''
Script that calculates the corner angles moving inward.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_angle_bisector.py

'''
import math
import numpy as np
from shapely.geometry import Polygon, Point

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_bisector(v0, v1, v2):
    """ 
    Returns the unit vector that bisects the angle formed by vertices v0, v1, v2.
    In other words if the corner of the two lines is 90 degrees it would return 45 degree point
    v1 is the vertex of the angle.
    """
    v0v1 = unit_vector(v0 - v1)
    v2v1 = unit_vector(v2 - v1)
    bisector = v0v1 + v2v1
    return unit_vector(bisector)

# Define polygon vertices and create the polygon
vertices = np.array([[15.9, 0], [12.9, 16.3], [23.3, 18.9], [22.3, 0.0]])
polygon = Polygon(vertices)

# Define step
step = 1.0

# Number of times to move inward
num_iterations = 8

# Initialize the list to store dot information
dots_info = []

# Option to turn visualization on or off
visualize = False

if visualize:
    import matplotlib.pyplot as plt
    plt.figure(figsize=(10, 10))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.fill(vertices[:, 0], vertices[:, 1], alpha=0.3)

# Add vertices to dots_info list with iteration as 0 and i as 1, 2, 3, 4 respectively.
for i, vertex in enumerate(vertices, start=1):
    dots_info.append([0, i, round(vertex[0], 2), round(vertex[1], 2)])

# Compute and plot dots
num_vertices = len(vertices)
for i in range(num_vertices):
    v0 = vertices[i - 1]  # Previous vertex
    v1 = vertices[i]  # Current vertex
    v2 = vertices[(i + 1) % num_vertices]  # Next vertex
    
    bisector = angle_bisector(v0, v1, v2)
    dot_position = v1 + step * bisector
    
    # Plot dots moving inward for num_iterations times
    #for iteration in range(num_iterations):
    for iteration in range(1, num_iterations):
        if not polygon.contains(Point(dot_position)):
            break  # Break if the dot is outside of the polygon
        
        if visualize:
            plt.plot(*dot_position, 'ro')
        
        # Append the information about the dot to the list
        dots_info.append([iteration, i, round(dot_position[0], 2), round(dot_position[1], 2)])
        
        dot_position = dot_position + step * bisector  # Move to the next position inside along the bisector

if visualize:
    plt.show()

# Print the dots_info list
# for info in dots_info:
#     print(info)

# After creating the dots_info list, sort it starting with the lower left corner and moving in a clockwise direction
dots_info.sort(key=lambda x: (x[0], x[1])) 

# Iterate over the dots_info list and calculate the angle for each point except the last one
for idx in range(len(dots_info) - 1):
    x1, y1 = dots_info[idx][2], dots_info[idx][3]  # current point
    x2, y2 = dots_info[idx + 1][2], dots_info[idx + 1][3]  # next point
    angle = math.atan2(y2 - y1, x2 - x1)
    if angle < 0:
    	angle += 2 * math.pi   # angle should be 0 to 2*pi(), not negative
    angle = round(angle, 2)
    dots_info[idx].append(angle)  # Append the angle to the current row in dots_info list

# Handle the last point separately
dots_info[-1].append(angle)  # Append the angle to the last row equal to the previous point

# Print the dots_info list
# for info in dots_info:
#     print(info)
# Open the file with write ('w') permission. 
# If the file doesn't exist, it will be created; if it exists, it will be truncated.
output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_squar_input3.txt"
# Loop through each coordinate. Write x, y, and angle separated by a space with a newline at the end
with open(output_file_waypoints, 'w') as file:
    for _, _, x, y, angle in dots_info:
        file.write(f"{x} {y} {angle}\n")
print("Done, output file:", output_file_waypoints)          