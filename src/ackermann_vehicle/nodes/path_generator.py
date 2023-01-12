#!/usr/bin/env python3

# https://pypi.org/project/dubins/

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

def generate_path(x0,y0,x1,y1,theta0,theta1):
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)
    turning_radius = 2.0
    step_size = 1.0

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


with open('/home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/waypoint.txt', 'r') as file:
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
        with open('generated_points.txt', 'w') as file:
            count = 0
            for p in drive_path:
                if count > 20:
                    px.append(p[0])
                    py.append(p[1])
                    pyaw.append(p[2])
                    count = 0
                count += 1

                drive_points.append([p[0],p[1]])
                file.write(str(p[0]) + " " + str(p[1]) + " " + str(p[2]) + "\n")

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

        
