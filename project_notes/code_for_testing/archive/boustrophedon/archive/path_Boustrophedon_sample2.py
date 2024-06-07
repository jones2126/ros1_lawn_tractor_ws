#!/usr/bin/env python
'''
Script that shows a simple example of Boustrophedon coverage path planner

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_Boustrophedon_sample2.py

'''
import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon, LineString

def rotate(cx, cy, x, y, angle):
    rad = np.radians(angle)
    cos_a = np.cos(rad)
    sin_a = np.sin(rad)
    x_new = (x - cx) * cos_a - (y - cy) * sin_a + cx
    y_new = (x - cx) * sin_a + (y - cy) * cos_a + cy
    return x_new, y_new


# Define polygon vertices
vertices = np.array([[15.9, 0], [12.9, 16.3], [23.3, 18.9], [22.3, 0.0]])
polygon = Polygon(vertices)

# Define the step and angle
step = 1.0  # Adjust step size as needed
angle = 85  # Angle in degrees

# Create bounding box
minx, miny, maxx, maxy = polygon.bounds
extend_factor = 3.0  # Increase if the lines are still not intersecting

# Calculate the line equations and intersections
start_y = miny - (maxx - minx) * extend_factor
end_y = maxy + (maxx - minx) * extend_factor
num_lines = int((end_y - start_y) / step) + 1

# Plotting Polygon
plt.figure(figsize=(10, 10))
plt.gca().set_aspect('equal', adjustable='box')
plt.fill(vertices[:,0], vertices[:,1], alpha=0.3)

# Plotting Stripes
for i in range(num_lines):
    y = start_y + i * step
    x1, y1 = minx - extend_factor * (maxx - minx), y
    x2, y2 = maxx + extend_factor * (maxx - minx), y
    
    # Rotate line points around the center of the bounding box
    cx, cy = (minx + maxx) / 2, (miny + maxy) / 2
    x1r, y1r = rotate(cx, cy, x1, y1, angle)
    x2r, y2r = rotate(cx, cy, x2, y2, angle)
    
    line = LineString([(x1r, y1r), (x2r, y2r)])
    intersection = line.intersection(polygon)
    
    # Optionally, visualize all lines, to verify if they are long enough
    # plt.plot([x1r, x2r], [y1r, y2r], 'r--', alpha=0.5)

    if intersection.is_empty or intersection.geom_type == 'Point':
        continue
    
    if intersection.geom_type == 'LineString':
        x, y = intersection.xy
        plt.plot(x, y, color='b')
    elif intersection.geom_type == 'MultiLineString':
        for geom in intersection.geoms:
            x, y = geom.xy
            plt.plot(x, y, color='b')

plt.show()
