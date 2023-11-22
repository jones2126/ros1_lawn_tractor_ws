#!/usr/bin/env python3  
'''
This script reads points from a CSV file, calculates the angle between consecutive points, and writes the points along with the calculated angles to a text file. 

Iterate over the waypoints list "points" and calculate the angle between the current point and the next point in the list
except the last one. Each point is represented as a tuple (x,y). The angle is calculated using the atan2 function from the 
math library, which returns a value in radians between −π and π.

The result is a list of tuples, where each tuple contains the x and y coordinates of a point and the angle to 
the next point. The angle is measured in radians and is updated to be between 0 and 2π.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_step3_v1.py
'''
import csv
import math
import matplotlib.pyplot as plt

def plot_graph(points):
    x_coords = [x for x, y, angle in points]
    y_coords = [y for x, y, angle in points]
    plt.scatter(x_coords, y_coords, color='red')
    plt.plot(x_coords, y_coords, 'b-')
    plt.axis('equal')
    plt.show()

def main():
    input_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step2.csv'
    output_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step3.txt'

    points = []
    with open(input_file, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # skip header
        for row in reader:
            x, y = map(float, row)
            points.append([x, y])

    extended_coordinates_list = []
    for idx in range(len(points) - 1):
        x1, y1 = points[idx]
        x2, y2 = points[idx + 1]
        angle = math.atan2(y2 - y1, x2 - x1)
        if angle < 0:
            angle += 2 * math.pi
        extended_coordinates_list.append((round(x1, 2), round(y1, 2), round(angle, 2)))

    # Handle the last point
    extended_coordinates_list.append((round(x2, 2), round(y2, 2), round(angle, 2)))

    lookahead = 2.5
    speed = 0.75
    with open(output_file, 'w') as file:
        for x, y, angle in extended_coordinates_list:
            file.write(f"{x} {y} {angle} {lookahead} {speed}\n")

    print("Done, output file:", output_file)
    plot_graph(extended_coordinates_list)

if __name__ == "__main__":
    main()
