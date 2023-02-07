#!/usr/bin/env python3
'''
calc_distance.py

Find the Euclidean distance between one two dimensional points.

ref: https://www.w3schools.com/python/ref_math_dist.asp
ref: https://stackoverflow.com/questions/31735499/calculate-angle-clockwise-between-two-points

'''
# Import math Library
import math
import numpy as np

def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return np.rad2deg((ang1 - ang2) % (2 * np.pi))  # result in degrees

def angle_rad(p1, p2):
	dx = p2[0] - p1[0] # Difference in x coordinates
	dy = p2[1] - p1[1] # Difference in y coordinates
	theta = math.atan2(dy, dx)    # Angle between p1 and p2 in radians
	return theta

# main routine
p = [0.00, 0.001]
q = [6.0, 0.008]  # East, just to the North
distance = math.dist(p, q)
dir_radians = angle_rad(p,q)
dir_degrees = dir_radians * 180 / math.pi
result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
print(result_string)

p = [0.00, 0.001]
q = [6.0, -0.008] # East, just to the South
distance = math.dist(p, q)
dir_radians = angle_rad(p,q)
dir_degrees = dir_radians * 180 / math.pi
result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
print(result_string)

p = [0.001, 0.001]
q = [-6.0, 0.004]  # West, just to the North
distance = math.dist(p, q)
dir_radians = angle_rad(p,q)
dir_degrees = dir_radians * 180 / math.pi
result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
print(result_string)

p = [0.001, 0.001]
q = [-6.0, -0.004] # West, just to the South
distance = math.dist(p, q)
dir_radians = angle_rad(p,q)
dir_degrees = dir_radians * 180 / math.pi
result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
print(result_string)

p = [0.001, 0.001]
q = [0.001, 6.0]
distance = math.dist(p, q)
dir_radians = angle_rad(p,q)
dir_degrees = dir_radians * 180 / math.pi 
result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
print(result_string)

p = [0.001, 0.001]
q = [0.001, -6.0]
distance = math.dist(p, q)
dir_radians = angle_rad(p,q)
dir_degrees = dir_radians * 180 / math.pi
result_string = "Distance between {0} and {1} is {2:4.1f} units and the direction is: {3:4.3f} in degrees and {4:4.3f} in radians".format(p, q, distance, dir_degrees, dir_radians)
print(result_string)