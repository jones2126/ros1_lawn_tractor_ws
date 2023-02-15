#!/usr/bin/env python3
'''
calc_coord.py

Multiple functions:
1. Find the Euclidean distance between one two dimensional points. input uses coordinates_a.txt in the format of "0.02, 0.002; 7.0, -0.009"
2. Calculate points around an axis to be used in a robot test mission

ref: https://www.w3schools.com/python/ref_math_dist.asp
ref: https://stackoverflow.com/questions/31735499/calculate-angle-clockwise-between-two-points

As background, ROS follows REP-103, an ENU system, and therefore any absolute orientations should be relative to a frame where X-positive 
points due east: http://www.ros.org/reps/rep-0103.html#coordinate-frame-conventions; East (0); North (1.57) West 270.1 degrees (3.14), West 269.9 (-3.14); South (-1.57)
Aircraft read in degrees with North reading zero and increasing with clockwise rotation. 


'''
from math import sin, cos, radians, pi, dist, atan2
import numpy as np
from tf.transformations import quaternion_from_euler

def angle_rad(p1, p2):
    dx = p2[0] - p1[0] # Difference in x coordinates
    dy = p2[1] - p1[1] # Difference in y coordinates
    theta = atan2(dy, dx)    # Angle between p1 and p2 in radians
    return theta

def point_pos(x1, y1, d, theta):
    # this calculates the new point based on the start x1, y1 position travelling d distance at theta angle
    # ref: https://stackoverflow.com/questions/48525583/get-a-points-position-from-an-angle-and-the-length-of-the-line
    x2 = round(x1 + d*cos(theta), 1)
    y2 = round(y1 + d*sin(theta), 1)
    return x2, y2

# main routine
print("processing points in coordinates_a.txt")
with open('/home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing/coordinates_a.txt', 'r') as file:
    content = file.readlines()
    content = [x.strip() for x in content]
    for line in content:
        points = line.split(";")
        #print(points[0], points[1])
        coord_1 = points[0].split(",")
        coord_2 = points[1].split(",")
        #print("first pt:(", float(coord_1[0]), ",", float(coord_1[1]),")", "second pt:(", float(coord_2[0]),",", float(coord_2[1]), ")")
        p = [float(coord_1[0]),float(coord_1[1])]
        q = [float(coord_2[0]),float(coord_2[1])]
        #print(p, q)
        distance = dist(p, q)
        dir_radians = angle_rad(p,q)
        dir_degrees = dir_radians * 180 / pi
        if dir_degrees < 0:
            dir_degrees = 360 + dir_degrees      
        result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
        print(result_string)        
print("end of points in coordinates_a.txt")

'''
Generate points that represent a circle with 6 points.  pi()/6 = 1.047 so the angles will be: target_angle_rad = [0, 1.047, 2.094, 3.142, -2.094, -1.047]

'''
print("calculate points for a circle")
starting_pt = [10.0, 10.0]
cir_diam = 5
#target_angle_rad = [0, 1.047, 2.094, 3.142, -2.094, -1.047]
#target_angle_rad = [0, 0.79, 1.57, 2.36, 3.14, 3.93, 4.71, 5.5]  # .785 increment ~pi()/4  8 point octogon
target_angle_rad = [0, 0.52, 1.05, 2.09, 2.62, 3.14, 3.67, 4.19, 4.71, 5.24, 5.76]  # .524 increment ~pi()/6  12 point Dodecagon
'''

'''
mission = []
for index, x in enumerate(target_angle_rad):
    new_point = point_pos(starting_pt[0], starting_pt[1], cir_diam, x)
    result_string = "New point {0} Going from {1},{2} at {3} radians generates the new point {4}".format(index, starting_pt[0], starting_pt[1], x, new_point)
    print(result_string)
    mission.append(new_point)
print("mission:", mission)
'''

Now I need to take the mission and calculate the angles, in radians, between points.  I can manually load the points into coordinates_a.txt and rerun the program
to accomplish the same thing.

example: [(15.0, 10.0), (12.5, 14.3), (7.5, 14.3), (5.0, 10.0), (7.5, 5.7), (12.5, 5.7)]
take the first and second; calculate angle; store 1st, 2nd and angle; change second to first; get next position
at the end use the first point as the destinateion for the last point to complete the circle

The format of the needed output is: (x, and y are the target pose and then angle in a quaternion)
    "create_pose(17.3,  9.3, 0.000, 0.000, 0.000, -0.365, 0.931),"
    "create_pose({x},   {y}, 0.000, q_x,   q_y,    q_z,   q_w),"
'''
mission_w_quaternion = []
for index, x in enumerate(mission):
    if index == 0:
        first_coord = x     # capturing the first point because a "from" and "to" set of points is needed
        last_coor = x       # saving this point so the circle can be completed at the end 
        pass
    else:
        dir_radians = angle_rad(first_coord, x)
        quat = quaternion_from_euler(0, 0, dir_radians)
        mission_step = [x[0], x[1], 0, round(quat[0], 4), round(quat[1], 4), round(quat[2], 4), round(quat[3], 4)]
        mission_w_quaternion.append(mission_step)
        # print(first_coord, x, round(dir_radians, 3), round(quat[0], 4), round(quat[1], 4), round(quat[2], 4), round(quat[3], 4))
        first_coord = x
dir_radians = angle_rad(x, last_coor)
quat = quaternion_from_euler(0, 0, dir_radians)
mission_step = [x[0], x[1], 0, round(quat[0], 4), round(quat[1], 4), round(quat[2], 4), round(quat[3], 4)]
mission_w_quaternion.append(mission_step)
#print(x, last_coor, round(dir_radians, 3), round(quat[0], 4), round(quat[1], 4), round(quat[2], 4), round(quat[3], 4))
print(mission_w_quaternion)
for x in mission_w_quaternion:
    result_string = "create_pose({0}, {1}, {2}, {3}, {4}, {5}, {6}),".format(x[0], x[1], x[2], x[3], x[4], x[5], x[6])
    print(result_string)