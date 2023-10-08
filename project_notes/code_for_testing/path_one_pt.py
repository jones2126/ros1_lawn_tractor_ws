#!/usr/bin/env python3
'''

python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_one_pt.py
'''

import math
import matplotlib.pyplot as plt

points = [[15.9, 0], [12.9, 16.3], [23.3, 18.9], [22.3, 0.0]]

# Calculating Point 5
x1, y1 = points[0]  # point 1
x4, y4 = points[3]  # point 4

dist = 1  # distance from point 1

if x1 != x4:
    slope = (y1 - y4) / (x1 - x4)
    delta_x = abs(1 / ((1 + slope ** 2) ** 0.5))
    delta_y = abs(slope * delta_x)
    x5 = x1 + delta_x if x1 < x4 else x1 - delta_x
    y5 = y1 + delta_y if y1 < y4 else y1 - delta_y
    points.append([x5, y5])  # Adding point 5 to the list

# Calculating Point 6
x2, y2 = points[1]  # point 2
x3, y3 = points[2]  # point 3

if x1 != x2 and x2 != x3:
    slope_12 = (y2 - y1) / (x2 - x1)
    slope_23 = (y3 - y2) / (x3 - x2)  
    
    c5 = y5 - slope_12 * x5
    c23 = y2 - slope_23 * x2  
    
    x_intercept = (c23 - c5) / (slope_12 - slope_23)
    y_intercept = slope_12 * x_intercept + c5 
    
    # Adjusting Point 6 to be 1 unit before the intercept point
    delta_x6 = 1 / ((1 + slope_23 ** 2) ** 0.5)
    delta_y6 = slope_23 * delta_x6
    x6 = x_intercept - delta_x6 if x_intercept > x3 else x_intercept + delta_x6
    y6 = y_intercept - delta_y6 if y_intercept > y3 else y_intercept + delta_y6

    points.append([x6, y6])  # Adding point 6 to the list

# Extracting x and y coordinates from the points list
x_coords = [x for x, y in points]
y_coords = [y for x, y in points]

# Plotting the points
plt.scatter(x_coords, y_coords, color='red')

# Plotting the lines
plt.plot(x_coords + [x_coords[0]], y_coords + [y_coords[0]], 'b-')

# Annotating the points
for i, (x, y) in enumerate(points):
    plt.annotate(f'Point {i + 1}', (x, y), textcoords="offset points", xytext=(0, 10), ha='center')

# Displaying the plot
plt.axis('equal')
plt.show()
