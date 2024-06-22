#!/usr/bin/env python3
# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_planner_stable/path_planning_waypoint_generator_dynamic_lookahead.py
'''
Read in a series of waypoints from the file waypoint.txt and expand them based on the step size and angle using the Dubins shortest path library.  The practical 
use of this is to have straight segments followed by a u-turn that is shaped like a key hole.

For an explanation for the input file can be found in the document 'Path Planning - Creating a waypoint plan using path_generator.odt' which can be 
found at https://github.com/jones2126/ros1_lawn_tractor_ws


references:
- credit to Matt Droter for an initial outline script
- https://pypi.org/project/dubins/

'''
import dubins
import math
import matplotlib.pyplot as plt
import csv 
import time

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

def generate_path(x0,y0,x1,y1,theta0,theta1,turning_radius,step_size):
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    continuous_path = dubins.shortest_path(q0, q1, turning_radius)
    calculated_path_segment, _ = continuous_path.sample_many(step_size) # takes the continuous Dubins path and samples it at intervals specified by step_size
    return calculated_path_segment

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def plot_path_from_csv(csv_file):
    return        

def calculate_curvature(p1, p2, p3):
    x1, y1 = p1[0], p1[1]
    x2, y2 = p2[0], p2[1]
    x3, y3 = p3[0], p3[1]

    # Calculate the semi-perimeter
    a = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    b = math.sqrt((x2 - x3)**2 + (y2 - y3)**2)
    c = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
    s = (a + b + c) / 2

    # Check for collinearity or near-collinearity
    area = s * (s - a) * (s - b) * (s - c)
    if area <= 0:
        return 0  # Curvature is zero if the points are collinear

    # Calculate the radius of the circumscribed circle
    radius = a * b * c / (4 * math.sqrt(area))

    # Calculate the curvature
    curvature = 1 / radius if radius != 0 else 0

    return curvature

def calculate_curvature_and_output_csv(drive_path, output_file_curvature_of_waypoints, alternate_lookahead, curvature_threshold):
  
    with open(output_file_curvature_of_waypoints, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'curvature', 'lookahead'])

        for i in range(1, len(drive_path) - 1):
            p1, p2, p3 = drive_path[i - 1], drive_path[i], drive_path[i + 1]
            curvature = calculate_curvature(p1, p2, p3)

            # Update lookahead distance based on curvature
            if curvature > curvature_threshold:
                lookahead = alternate_lookahead
            else:
                lookahead = p2[3]  # Use existing lookahead if curvature is not above threshold

            # Update the tuple in drive_path
            drive_path[i] = (p2[0], p2[1], p2[2], lookahead, p2[4])

            # Write data to CSV
            writer.writerow([p2[0], p2[1], curvature, lookahead])
    print(f"file written with curvature data: {output_file_curvature_of_waypoints}")
    return drive_path 

def main():
    def read_waypoints(input_file_waypoints):
        with open(input_file_waypoints, 'r') as file:
            content = file.readlines()
        return [x.strip() for x in content]

    def generate_drive_path(waypoints, turning_radius, step_size, lookahead, speed):
        drive_path = []
        x0, y0, theta0 = 0.0, 0.0, 0.0
        for idx, line in enumerate(waypoints):
            points = line.split()
            x1, y1, theta1 = map(float, points[:3])
            if idx == 0:
                x0, y0, theta0 = x1, y1, theta1   # For the first line, just set the initial x, y, theta
                continue
            # Generate path from the previous to the current waypoint
            calculated_path_segment = generate_path(x0, y0, x1, y1, theta0, theta1, turning_radius, step_size)
            for segment in calculated_path_segment:
                segment += (lookahead, speed)   # Appending lookahead and speed to each point in the path
                drive_path.append(segment)
            x0, y0, theta0 = x1, y1, theta1     # Updating the start point for the next segment
        return drive_path

    def write_output_file(output_file_waypoints, drive_path):
        total_count = 0
        with open(output_file_waypoints, 'w') as file:
            for p in drive_path:
                total_count += 1
                file.write(f"{p[0]} {p[1]} {p[2]} {p[3]} {p[4]}\n")
        return total_count

    def plot_points(input_file_waypoints, drive_path, alternate_lookahead, lookahead):
        px, py, pyaw = [], [], []
        count = 0

        for i in range(len(drive_path) - 1):
            print(f"in the loop, i: {i}")
            p1, p2 = drive_path[i], drive_path[i+1]

            # Decide color based on lookahead value
            if p1[3] == alternate_lookahead:  # Check if lookahead is 1.0
                color = 'red'  # Choose a color for lookahead 1.0
                label = 'Lookahead ' + str(alternate_lookahead)
            else:
                color = 'blue'  # Default color for other lookahead values
                label = 'Lookahead ' + str(lookahead)

            # Plot line segment
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color=color)

            # For decluttering the visualization
            if count > 20:
                px.append(p1[0])
                py.append(p1[1])
                pyaw.append(p1[2])
                count = 0
            count += 1

            # Plotting arrows (optional, based on your existing logic)
            for (ix, iy, iyaw) in zip(px, py, pyaw):
                plot_arrow(ix, iy, iyaw, fc="b")

        # Plotting waypoints
        print(f"about to open file: {input_file_waypoints}")
        with open(input_file_waypoints, 'r') as file:
            waypoints = file.readlines()
        
        for idx, waypoint in enumerate(waypoints):
            #x, y, _ = map(float, waypoint.split())
            x, y, _, _, _ = map(float, waypoint.split())
            plt.plot(x, y, 'ro', markersize=5)
            plt.text(x + 0.1, y - 0.1, str(idx + 1), fontsize=6, color='green')

        plt.title(f'Continuous Curvature Path from: path_planning_waypoint_generator.py')
        plt.figtext(0.5, 0.01, f'Input: {input_file_waypoints}', ha='center', fontsize=8, color='gray')
        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()



    # main routine
    turning_radius=1.3
    step_size=0.3
    lookahead=2.5
    alternate_lookahead=1.0
    curvature_threshold=0.02
    speed=0.75

    working_directory = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/"
    input_file_waypoints = working_directory + "Site_01_transition_061924.txt"
    #/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophedon_line_segments_input_path_for_generator.txt
    output_file_curvature_of_waypoints = working_directory + "test_PID_curvature_of_waypoints.csv"
    output_file_waypoints = working_directory + "test_generator_output.txt"
    waypoints = read_waypoints(input_file_waypoints)
    drive_path = generate_drive_path(waypoints, turning_radius, step_size, lookahead, speed)    # Process 1: Path Generation
    print("done with generate_drive_path")
    # Call the function to calculate curvature and output to CSV
    drive_path = calculate_curvature_and_output_csv(drive_path, output_file_curvature_of_waypoints, alternate_lookahead, curvature_threshold)
    print("done with calculate_curvature_and_output_csv")
    total_count = write_output_file(output_file_waypoints, drive_path)                          # Process 2: Output File Creation and Data Appending
    print(f"Output file: {output_file_waypoints}")
    print("Record count:", total_count, "turning_radius:", turning_radius, "step_size:", step_size)


    plot_points(input_file_waypoints, drive_path, alternate_lookahead, lookahead)                          # Process 3: Plotting Points

    # Call the plotting function
    #plot_path_from_csv('path_data.csv')
    print("eoj")

if __name__ == "__main__":
    main()
