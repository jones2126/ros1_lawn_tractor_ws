#!/usr/bin/env python3
'''
Script that calculates the corner angles moving inward.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_polygon_clockwise2.py

'''

import math
import matplotlib.pyplot as plt

def calculate_point_five(points, path_width):
    x1, y1 = points[0]  # point 1
    x4, y4 = points[3]  # point 4
    
    if x1 != x4:
        slope = (y1 - y4) / (x1 - x4)
        delta_x = abs(path_width / ((1 + slope ** 2) ** 0.5))
        delta_y = abs(slope * delta_x)
        x5 = x1 + delta_x if x1 < x4 else x1 - delta_x
        y5 = y1 + delta_y if y1 < y4 else y1 - delta_y
        
        points.append([x5, y5])  # Adding point 5 to the list
    return points

def calculate_interior_points(points, slopes, path_width, iterations):
    for i in range(iterations):
        last_point_index = len(points) - 1
        slope_index = (i + 1) % len(slopes)  # To avoid index out of range error
        
        # Getting coordinates of the last point added to the list and its slope
        xl, yl = points[last_point_index]
        slope_l = slopes[slope_index] if slopes[slope_index] != 'undefined' else slopes[0]
        
        # Getting the coordinates of adjacent points for intersection calculation
        xa, ya = points[slope_index]
        xb, yb = points[(slope_index + 1) % len(points)]
        
        if xa == xb:  # line segment is vertical
            xi = xa
            yi = slope_l * (xi - xl) + yl  # y = mx + c, where m is the slope and c is the y-intercept
        else:
            slope_ab = (yb - ya) / (xb - xa)  # Slope of the line segment
            if slope_ab == slope_l:  # Check if the lines are parallel
                # You need to define the logic for parallel lines, below is an example.
                xi = xl + path_width  # Adjust according to your requirement for parallel lines
                yi = yl
            else:
                cab = ya - slope_ab * xa  # y = mx + c => c = y - mx
                cl = yl - slope_l * xl  # y-intercept of the line extending from the last point
                
                # Finding the intersection of the two lines
                xi = (cab - cl) / (slope_l - slope_ab)
                yi = slope_l * xi + cl  # y = mx + c
        
        # Adjusting Intersection Point to be path_width unit before the intercept point
        delta_x = abs(path_width / ((1 + slope_l ** 2) ** 0.5))
        delta_y = abs(slope_l * delta_x)
        xi = xi - delta_x if xi > xl else xi + delta_x
        yi = yi - delta_y if yi > yl else yi + delta_y
        
        points.append([xi, yi])  # Adding new point to the list
    
    return points


def calculate_slopes(points):
    slopes = []
    num_points = len(points)
    for i in range(num_points - 1):  # Exclude the last to the first point segment
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        if x2 - x1 != 0:
            slope = (y2 - y1) / (x2 - x1)
        else:
            slope = 'undefined'  # For vertical line segments
        slopes.append(slope)
    return slopes

def plot_graph(points):
    # Extracting x and y coordinates from the points list
    x_coords = [x for x, y in points]
    y_coords = [y for x, y in points]
    
    # Plotting the points
    plt.scatter(x_coords, y_coords, color='red')
    
    # Plotting line segments individually
    num_points = len(points)
    for i in range(num_points - 1):
        plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], 'b-')

    # If you don’t want a line between the last and the first point, remove the below line
    # if num_points > 1:
    #     plt.plot([points[num_points - 1][0], points[0][0]], [points[num_points - 1][1], points[0][1]], 'b-')
    
    # Annotating the points
    for i, (x, y) in enumerate(points):
        plt.annotate(f'Point {i + 1}', (x, y), textcoords="offset points", xytext=(0,10), ha='center')
    
    plt.axis('equal')
    plt.show()


def main():
    points = [[15.9, 0], [12.9, 16.3], [23.3, 18.9], [22.3, 0.0]]
    path_width = 1  # You can change this value as per your need.
    points = calculate_point_five(points, path_width)
    slopes = calculate_slopes(points)
    print("Slopes of the line segments:", slopes)

    iterations = 5  # You can adjust this value as per your requirement
    points = calculate_interior_points(points, slopes, path_width, iterations)
    
    plot_graph(points)


if __name__ == "__main__":
    main()
