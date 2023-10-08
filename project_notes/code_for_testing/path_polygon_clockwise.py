#!/usr/bin/env python3
'''
Script that calculates the corner angles moving inward.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_polygon_clockwise.py

'''

import math
import matplotlib.pyplot as plt

def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)
    return angle

def adjust_angle(angle):
    return angle if angle >= 0 else (2 * math.pi + angle)

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

def calculate_generic_point(points, slopes, path_width, index_for_slope, index_for_intersection_points):
    # Calculates coordinates of the next point for the mission based on the inputs
    x_previous, y_previous = points[-1]  # Gets the last point from the current list of points
    slope = slopes[index_for_slope] # get the slope from slopes list
    x1, y1 = points[index_for_intersection_points[0]]  # Get coordinates of the intersecting line segments
    x2, y2 = points[index_for_intersection_points[1]]
    if slope == 0:  # check if the line is horizontal
        y_new = y_previous
        if x1 == x2:  # The intersecting line segment is vertical
            x_new = x1
        else:  # The intersecting line segment is not vertical
            slope_inter = (y2 - y1) / (x2 - x1)
            c_inter = y1 - slope_inter * x1
            x_new = (y_new - c_inter) / slope_inter
    
    else:  # The line extending from the last point is not horizontial
        c_previous = y_previous - slope * x_previous
        if x1 == x2:  # The intersecting line segment is vertical
            x_new = x1
            y_new = slope * x_new + c_previous
        else:  # None of the lines are vertical
            slope_inter = (y2 - y1) / (x2 - x1)
            c_inter = y1 - slope_inter * x1
            x_new = (c_inter - c_previous) / (slope - slope_inter)
            y_new = slope * x_new + c_previous
    
    # Calculate the new point which is 'path_width' before the intercept point
    delta_x = abs(path_width / ((1 + slope ** 2) ** 0.5))
    delta_y = abs(slope * delta_x)
    x_new = x_new - delta_x if x_new > x_previous else x_new + delta_x
    y_new = y_new - delta_y if y_new > y_previous else y_new + delta_y
    
    points.append([x_new, y_new])  # Adding the new point to the list
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
    x_coords = [x for x, y in points]    # Extracting x and y coordinates from the points list
    y_coords = [y for x, y in points]
    plt.scatter(x_coords, y_coords, color='red')     # Plotting the points
    
    # Plotting line segments individually
    num_points = len(points)
    for i in range(num_points - 1):
        plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], 'b-')

    # Annotating the points
    for i, (x, y) in enumerate(points):
        plt.annotate(f'Point {i + 1}', (x, y), textcoords="offset points", xytext=(0,10), ha='center')
    
    plt.axis('equal')
    plt.show()


def main():
    points = [[15.9, 0], [12.9, 16.3], [23.3, 18.9], [22.3, 0.0]]
    path_width = 1  # my cutting deck is 42" wide (1.067 meters)
    points = calculate_point_five(points, path_width)
    slopes = calculate_slopes(points)
    print("Slopes of the line segments:", slopes)
    points = calculate_generic_point(points, slopes, path_width, 0, [1, 2])  # point 6    
    points = calculate_generic_point(points, slopes, path_width, 1, [2, 3])  # point 7    
    points = calculate_generic_point(points, slopes, path_width, 2, [3, 4])  # point 8
    points = calculate_generic_point(points, slopes, path_width, 3, [4, 5])  # point 9
    points = calculate_generic_point(points, slopes, path_width, 0, [5, 6])  # point 10
    points = calculate_generic_point(points, slopes, path_width, 1, [6, 7])  # point 11
    points = calculate_generic_point(points, slopes, path_width, 2, [7, 8])  # point 12
   # points = calculate_generic_point(points, slopes, path_width, 3, [8, 9])  # point 13   
   # points = calculate_generic_point(points, slopes, path_width, 0, [9, 10])  # point 14
   # points = calculate_generic_point(points, slopes, path_width, 1, [10, 11])  # point 15

    # Add the origin coordinate
    origin = [7.7, -3.0]
    points.insert(0, origin)

    print("List of waypoints:")
    for coord in points:
        print(coord)

    # Iterate over the waypoints list and calculate the angle for each point except the last one
    extended_coordinates_list = []
    print('Numbr of points:', len(points))
    for idx in range(len(points) - 1):
        print('IDX:', idx)
        x1, y1 = points[idx][0], points[idx][1]  # current point
        x2, y2 = points[idx + 1][0], points[idx + 1][1]  # next point
        angle = math.atan2(y2 - y1, x2 - x1)
        if angle < 0:
            angle += 2 * math.pi   # angle should be 0 to 2*pi(), not negative
        #angle = round(angle, 2)
        #points[idx].append(angle)  # Append the angle to the current row in points list
        extended_coordinates_list.append((round(x1,2), round(y1,2), round(angle,2)))

    # Handle the last point separately.  Its really a guess on the angle
    extended_coordinates_list.append((round(x2,2), round(y2,2), round(angle,2)))
    # Append the angle to the last row equal to the previous point
    # Output a file that will be used by the path genrator
    output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_squar_input3.txt"
    # Loop through each coordinate. Write x, y, and angle separated by a space with a newline at the end
    with open(output_file_waypoints, 'w') as file:
        for x, y, angle in extended_coordinates_list:
            file.write(f"{x} {y} {angle}\n")
    print("Done, output file:", output_file_waypoints)   

    plot_graph(points)        

if __name__ == "__main__":
    main()
