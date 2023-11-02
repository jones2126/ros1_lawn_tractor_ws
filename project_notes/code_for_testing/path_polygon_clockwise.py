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

# the math is unique to calculate the first inside point after the 4 provided points
def calculate_point_five(points, path_width):
    x1, y1 = points[0]  # point 2 in the finished path (first point of the 4 provided corners)
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
    # points = [[19.2, -40.9], [-6.9, -52.3], [-29.8, -22.7], [4.3, -4.5]]
    points = [[4.0, -4.6], [29.5, 2.7], [21.0, 35.0], [-9.1, 26.3]]             # point 2, 3, 4, 5
    path_width = 1  # my cutting deck is 42" wide (1.067 meters)
    points = calculate_point_five(points, path_width)
    slopes = calculate_slopes(points)
    print("Slopes of the line segments:", slopes)
    points = calculate_generic_point(points, slopes, path_width, 0, [1, 2])     # point 7    
    points = calculate_generic_point(points, slopes, path_width, 1, [2, 3])     # point 8    
    points = calculate_generic_point(points, slopes, path_width, 2, [3, 4])     # point 9
    points = calculate_generic_point(points, slopes, path_width, 3, [4, 5])     # point 10
    points = calculate_generic_point(points, slopes, path_width, 0, [5, 6])     # point 11
    points = calculate_generic_point(points, slopes, path_width, 1, [6, 7])     # point 12
    points = calculate_generic_point(points, slopes, path_width, 2, [7, 8])     # point 13
    points = calculate_generic_point(points, slopes, path_width, 3, [8, 9])     # point 14   
    #points = calculate_generic_point(points, slopes, path_width, 0, [9, 10])    # point 15
    #points = calculate_generic_point(points, slopes, path_width, 1, [10, 11])   # point 16
    #points = calculate_generic_point(points, slopes, path_width, 2, [11, 12])   # point 17
    #points = calculate_generic_point(points, slopes, path_width, 3, [12, 13])   # point 18
    #points = calculate_generic_point(points, slopes, path_width, 0, [13, 14])   # point 19
    #points = calculate_generic_point(points, slopes, path_width, 1, [14, 15])   # point 20
    #points = calculate_generic_point(points, slopes, path_width, 2, [15, 16])   # point 21
    #points = calculate_generic_point(points, slopes, path_width, 3, [16, 17])   # point 22
    #points = calculate_generic_point(points, slopes, path_width, 0, [17, 18])   # point 23
    #points = calculate_generic_point(points, slopes, path_width, 1, [18, 19])   # point 24
    #points = calculate_generic_point(points, slopes, path_width, 2, [19, 20])   # point 25
    #points = calculate_generic_point(points, slopes, path_width, 3, [20, 21])   # point 26
    #points = calculate_generic_point(points, slopes, path_width, 0, [21, 22])   # point 27
    #points = calculate_generic_point(points, slopes, path_width, 1, [22, 23])   # point 28
    #points = calculate_generic_point(points, slopes, path_width, 2, [23, 24])   # point 29
    #points = calculate_generic_point(points, slopes, path_width, 3, [24, 25])   # point 30
    #points = calculate_generic_point(points, slopes, path_width, 0, [25, 26])   # point 31
    #points = calculate_generic_point(points, slopes, path_width, 1, [26, 27])   # point 32
    #points = calculate_generic_point(points, slopes, path_width, 2, [27, 28])   # point 33
    #points = calculate_generic_point(points, slopes, path_width, 3, [28, 29])   # point 34
    #points = calculate_generic_point(points, slopes, path_width, 0, [29, 30])   # point 35
    #points = calculate_generic_point(points, slopes, path_width, 1, [30, 31])   # point 36
    #points = calculate_generic_point(points, slopes, path_width, 2, [31, 32])   # point 37
    #points = calculate_generic_point(points, slopes, path_width, 3, [32, 33])   # point 38
    #points = calculate_generic_point(points, slopes, path_width, 0, [33, 34])   # point 39
    #points = calculate_generic_point(points, slopes, path_width, 1, [34, 35])   # point 40
    #points = calculate_generic_point(points, slopes, path_width, 2, [35, 36])   # point 41
    #points = calculate_generic_point(points, slopes, path_width, 3, [36, 37])   # point 42
    #points = calculate_generic_point(points, slopes, path_width, 0, [37, 38])   # point 43
    #points = calculate_generic_point(points, slopes, path_width, 1, [38, 39])   # point 44
    #points = calculate_generic_point(points, slopes, path_width, 2, [39, 40])   # point 45
    #points = calculate_generic_point(points, slopes, path_width, 3, [40, 41])   # point 46
    #points = calculate_generic_point(points, slopes, path_width, 0, [41, 42])   # point 47
    #points = calculate_generic_point(points, slopes, path_width, 1, [42, 43])   # point 48
    #points = calculate_generic_point(points, slopes, path_width, 2, [43, 44])   # point 49
    #points = calculate_generic_point(points, slopes, path_width, 3, [44, 45])   # point 50
    #points = calculate_generic_point(points, slopes, path_width, 0, [45, 46])   # point 51
    #points = calculate_generic_point(points, slopes, path_width, 1, [46, 47])   # point 52
    # points = calculate_generic_point(points, slopes, path_width, 2, [47, 48])   # point 53
    # points = calculate_generic_point(points, slopes, path_width, 3, [48, 49])   # point 54
    # points = calculate_generic_point(points, slopes, path_width, 0, [49, 50])   # point 55
   # points = calculate_generic_point(points, slopes, path_width, 1, [50, 51])   # point 56
   # points = calculate_generic_point(points, slopes, path_width, 2, [51, 52])   # point 57
   # points = calculate_generic_point(points, slopes, path_width, 3, [52, 53])   # point 58
   # points = calculate_generic_point(points, slopes, path_width, 0, [53, 54])   # point 59
   # points = calculate_generic_point(points, slopes, path_width, 1, [54, 55])   # point 60
   # points = calculate_generic_point(points, slopes, path_width, 2, [55, 56])   # point 61
   # points = calculate_generic_point(points, slopes, path_width, 3, [56, 57])   # point 62
   # points = calculate_generic_point(points, slopes, path_width, 0, [57, 58])   # point 63
   # points = calculate_generic_point(points, slopes, path_width, 1, [58, 59])   # point 64
   # points = calculate_generic_point(points, slopes, path_width, 2, [59, 60])   # point 65
   # points = calculate_generic_point(points, slopes, path_width, 3, [60, 61])   # point 66
   # points = calculate_generic_point(points, slopes, path_width, 0, [61, 62])   # point 67
   # points = calculate_generic_point(points, slopes, path_width, 1, [62, 63])   # point 68
   # points = calculate_generic_point(points, slopes, path_width, 2, [63, 64])   # point 69
   # points = calculate_generic_point(points, slopes, path_width, 3, [64, 65])   # point 70
   # points = calculate_generic_point(points, slopes, path_width, 0, [65, 66])   # point 71
   # points = calculate_generic_point(points, slopes, path_width, 1, [66, 67])   # point 72
   # points = calculate_generic_point(points, slopes, path_width, 2, [67, 68])   # point 73
   # points = calculate_generic_point(points, slopes, path_width, 3, [68, 69])   # point 74
   # points = calculate_generic_point(points, slopes, path_width, 0, [69, 70])   # point 75
   # points = calculate_generic_point(points, slopes, path_width, 1, [70, 71])   # point 76
   # points = calculate_generic_point(points, slopes, path_width, 2, [71, 72])   # point 77
   # points = calculate_generic_point(points, slopes, path_width, 3, [72, 73])   # point 78        

    # Add the origin coordinate
    origin = [0.0, 0.0]  # point 1
    points.insert(0, origin)

    print("List of waypoints:")
    for coord in points:
        print(coord)

'''
Iterate over the waypoints list "points" and calculate the angle between the current point and the next point in the list
except the last one. Each point is epresented as a tuple (x,y). The angle is calculated using the atan2 function from the 
math library, which returns a value in radians between −π and π.

The result is a list of tuples, where each tuple contains the x and y coordinates of a point and the angle to 
the next point. The angle is measured in radians and is updated to be between 0 and 2π.
'''
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
    output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_62_step1.txt"
    # Loop through each coordinate. Write x, y, and angle separated by a space with a newline at the end
    lookahead = 2.5
    speed = 0.75    
    with open(output_file_waypoints, 'w') as file:
        for x, y, angle in extended_coordinates_list:
            #file.write(f"{x} {y} {angle}\n")
            file.write(f"{x} {y} {angle} {lookahead} {speed}\n")
    print("Done, output file:", output_file_waypoints)   

    plot_graph(points)        

if __name__ == "__main__":
    main()
