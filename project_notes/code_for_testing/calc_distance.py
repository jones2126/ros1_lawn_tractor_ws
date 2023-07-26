#!/usr/bin/env python3
'''

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/calc_distance.py

Find the Euclidean distance between one two dimensional points.

ref: https://www.w3schools.com/python/ref_math_dist.asp
ref: https://stackoverflow.com/questions/31735499/calculate-angle-clockwise-between-two-points

'''
# Import math Library
import math
import numpy as np
from geopy.distance import geodesic
from geopy import Point


def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return np.rad2deg((ang1 - ang2) % (2 * np.pi))  # result in degrees

def angle_rad(p1, p2):
	dx = p2[0] - p1[0] # Difference in x coordinates
	dy = p2[1] - p1[1] # Difference in y coordinates
	theta = math.atan2(dy, dx)    # Angle between p1 and p2 in radians
	return theta

from math import sin, cos, radians, pi
def point_pos(x0, y0, d, theta):
	# ref: https://stackoverflow.com/questions/23280636/python-find-a-x-y-coordinate-for-a-given-point-b-using-the-distance-from-the-po
    theta_rad = pi/2 - radians(theta)
    return x0 + d*cos(theta_rad), y0 + d*sin(theta_rad)
    # option 2, https://stackoverflow.com/questions/48525583/get-a-points-position-from-an-angle-and-the-length-of-the-line
    #(x2,y2) = (x1 + line_length*cos(angle),y1 + line_length*sin(angle))

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

# calculate the starting position based on the ending position, angle and distance travelled
end_position = Point(40.34524742, -80.12890872666667)  # The ending GPS position
distance = 4.74 / 1000  # The distance travelled in kilometers

# The compass heading
# Note: In geopy, bearings are measured clockwise from North (i.e., standard compass bearings),
# so we don't need to adjust the heading.
heading = 185

# Calculate the starting GPS position
start_position = geodesic(kilometers=distance).destination(point=end_position, bearing=(heading + 180) % 360)

# Get the latitude and longitude in decimal degrees, rounded to 6 decimal places
start_latitude = round(start_position.latitude, 7)
start_longitude = round(start_position.longitude, 7)

print("Latitude:", start_latitude)
print("Longitude:", start_longitude)