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

def project_lat_lon(start_lat, start_lon, distance_meters, heading):
    """
    Calculate the ending GPS position based on the starting position, angle, and distance traveled.
    
    Args:
    - start_lat (float): Latitude of the starting position in decimal degrees.
    - start_lon (float): Longitude of the starting position in decimal degrees.
    - distance_meters (float): Distance traveled in meters.
    - heading (float): Compass heading in degrees. North is 0, East is 90, etc.
    
    Returns:
    - tuple: Ending latitude and longitude as a tuple of floats.
    """
    
    start_position = Point(start_lat, start_lon)
    distance_km = distance_meters / 1000  # Convert distance to kilometers
    
    # Calculate the ending GPS position
    end_position = geodesic(kilometers=distance_km).destination(point=start_position, bearing=heading)
    
    # Get the latitude and longitude in decimal degrees, rounded to 7 decimal places
    end_latitude = round(end_position.latitude, 7)
    end_longitude = round(end_position.longitude, 7)
    
    return end_latitude, end_longitude


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


# Define the radian values
radians = [0.001, 1.569, 1.571, 3.139, -3.139, -1.571, -1.569, -0.001]

# Convert radians to degrees
degrees = [round(math.degrees(rad), 2) for rad in radians]

print(degrees)

# Calculate new lat/lon using starting position, distance and heading
start_lat = 40.345300971941796
start_lon = -80.1289413912553
distance_meters = 175 * .0254
heading = 355

end_lat, end_lon = project_lat_lon(start_lat, start_lon, distance_meters, heading)
result_string = ("Starting lat/lon: ({0:.8f}, {1:.8f}), heading: {2}°, distance: {3:.2f} meters; "
                 "Ending lat/lon: ({4:.8f}, {5:.8f})").format(start_lat, start_lon, heading, distance_meters, end_lat, end_lon)
print(result_string)
