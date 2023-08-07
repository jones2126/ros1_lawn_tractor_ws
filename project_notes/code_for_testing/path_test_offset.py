#!/usr/bin/env python3
'''
Testing program to verify offset is calculated correctly
Premise: With a yaw of 0, the robot is facing in the positive X-direction in the coordinate 
system which means the the GPS coordinates should be further along the positive X-axis compared
to the base link coordinates.

To run: $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_test_offset.py
'''

import geonav_transform.geonav_conversions as gc
import math

# Given origin
GPS_origin_lat = 40.3452899
GPS_origin_lon = -80.1289039

# Raw GPS coordinates
current_lat = 40.345273
current_lon = -80.128643

# Yaw (heading) in radians
yaw_radians = 0.0

# Offset of the GPS from base link
x_offset = 0.5
y_offset = 0

x_gps, y_gps = gc.ll2xy(current_lat, current_lon, GPS_origin_lat, GPS_origin_lon)
x_offset_rotated = x_offset * math.cos(yaw_radians) - y_offset * math.sin(yaw_radians)
y_offset_rotated = x_offset * math.sin(yaw_radians) + y_offset * math.cos(yaw_radians)
x_base_link = x_gps - x_offset_rotated
y_base_link = y_gps - y_offset_rotated

# Rough approximate of expected location based on field test
expected_x = 22.1
expected_y = -1.7

difference_x = expected_x - x_base_link
difference_y = expected_y - y_base_link

print("GPS coordinates (X, Y):", round(x_gps, 2), round(y_gps, 2))
print("Base link coordinates (X, Y):", round(x_base_link, 2), round(y_base_link, 2))
print("Difference from expected location (X, Y):", round(difference_x, 2), round(difference_y, 2))
