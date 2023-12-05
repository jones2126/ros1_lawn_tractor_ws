#!/usr/bin/env python3
# path_planning_plot.py
'''
'''
#import dubins
#import math
import matplotlib.pyplot as plt

output_file_waypoints = "/home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_hay_cutting_E20X10L.txt_waypoints.txt"
px = []
py = []
pyaw = []
with open(output_file_waypoints, 'r') as file:
    content = file.readlines()
    content = [x.strip() for x in content]
    for line in content:
        points = line.split()
        x1 = float(points[0])
        y1 = float(points[1])
        px.append(x1)
        py.append(y1)
print(px)
print(py)
plt.legend()
plt.grid(True)
plt.plot(px, py)
plt.title("Cutting Hay Path")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()