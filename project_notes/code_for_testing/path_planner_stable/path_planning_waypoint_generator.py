#!/usr/bin/env python3
# credit for original script I believe goes to Matt Droter
#
# https://pypi.org/project/dubins/
# this will read in a series of waypoints from the file waypoint.txt and create a path with additional points using the dubins library
# there is a spreadsheet (vehicle_dimensions_helper.ods) to help with simple paths (e.g. for testing)
# there is additional explanation in the document Path Planning - Creating a waypoint plan using path_generator.odt
# all can be found at https://github.com/jones2126/ros1_lawn_tractor_ws
#
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_planner_stable/path_planning_waypoint_generator.py
import dubins
import math
import matplotlib.pyplot as plt

# Vairables
show_animation = True
x0 = 0.0
y0 = 0.0
theta0 = 0.0
x1 = 0.0
y1 = 0.0
theta1 = 0.0
speed = 0.0
drive_path = []
drive_points = []

start_x = 0.0
start_y = 0.0
start_yaw = 0.0
total_count = 0 
turning_radius = 1.3    # originally 2
step_size = 0.1         # originally 1

def generate_path(x0,y0,x1,y1,theta0,theta1):
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    path = dubins.shortest_path(q0, q1, turning_radius)
    configurations, _ = path.sample_many(step_size)
    return configurations

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

# main routine
#
#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/square.txt"
#output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/generated_points_for_square.txt"
#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/435_pine_valley_test2.txt"
#output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/435_PV_test2_GP.txt"
#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_square_2.txt"
#output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_green_marks2_output_01.txt"
#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_green_marks2.txt"
#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_3turnoverlap_input_v2.txt"
#output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_3turnoverlap_output_v2.txt"
#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_1_uturn.txt"
#output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_1_uturn_output.txt"

#input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_squar_input3.txt"
input_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_uturn_input4.txt"
output_file_waypoints = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_uturn_output.txt"
print("Output file:", output_file_waypoints)


with open(input_file_waypoints, 'r') as file:
    content = file.readlines()
    content = [x.strip() for x in content]
    for line in content:
        points = line.split()
        if line == content[0]:
            # Start of file just load waypoint
            # then get next line
            x1 = float(points[0])
            y1 = float(points[1])
            theta1 = float(points[2])
            #speed = float(points[3])
            
        else:
            #if line == content[len(content)-1]:
            #    print('end of file')
            x1 = float(points[0])
            y1 = float(points[1])
            theta1 = float(points[2])
            #speed = float(points[3])
            path = generate_path(x0,y0,x1,y1,theta0,theta1)
            for i in path:
                i = i + (speed,)
                drive_path.append(i)

        # The final waypoint becomes the start of the
        # next segment.
        x0 = x1
        y0 = y1
        theta0 = theta1
        #speed = speed


    if show_animation:
        px = []
        py = []
        pyaw = []
        with open(output_file_waypoints, 'w') as file:
            count = 0
            for p in drive_path:
                if count > 20:     # I think this is done to declutter the visualization
                    px.append(p[0])
                    py.append(p[1])
                    pyaw.append(p[2])
                    count = 0
                count += 1
                total_count += 1 

                drive_points.append([p[0],p[1]])
                file.write(str(p[0]) + " " + str(p[1]) + " " + str(p[2]) + str(p[3]) + "\n")
        print("Record count:", total_count, "turning_radius:", turning_radius, "step_size:", step_size)
        plt.plot(0,0, label="final course")
        plt.plot(*zip(*drive_points))

        # plotting
        #plot_arrow(start_x, start_y, start_yaw)
        #plot_arrow(end_x, end_y, end_yaw)

        for (ix, iy, iyaw) in zip(px, py, pyaw):
            plot_arrow(ix, iy, iyaw, fc="b")

        plt.legend()
        plt.grid(True)
        plt.axis("equal")
        plt.show()

        
