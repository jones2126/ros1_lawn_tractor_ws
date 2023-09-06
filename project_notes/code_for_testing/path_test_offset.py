#!/usr/bin/env python3
'''
Testing program to verify offset is calculated correctly
Premise: With a yaw of 0, the robot is facing in the positive X-direction in the coordinate 
system which means the the GPS coordinates should be further along the positive X-axis compared
to the base link coordinates.  Since my GPS is also slightly to the right of the centerline
the Y-direction would be slightly negative.

To run: $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_test_offset.py
'''

import geonav_transform.geonav_conversions as gc
import math

# Given origin
GPS_origin_lat = 40.3452899
GPS_origin_lon = -80.1289039

# Assume these are the raw GPS coordinates read by the GPS.
current_GPS_lat = 40.345273
current_GPS_lon = -80.128643

# Offset of the GPS from base link based on the physical implementation of the GPS
x_offset = 0.51
y_offset = -0.1

# List of yaw values to test
yaw_values = [0, 1.57, 3.14, -1.57]

for yaw_radians in yaw_values:
    x_gps, y_gps = gc.ll2xy(current_GPS_lat, current_GPS_lon, GPS_origin_lat, GPS_origin_lon)
    x_offset_rotated = x_offset * math.cos(yaw_radians) - y_offset * math.sin(yaw_radians)
    y_offset_rotated = x_offset * math.sin(yaw_radians) + y_offset * math.cos(yaw_radians)
    x_base_link = x_gps - x_offset_rotated
    y_base_link = y_gps - y_offset_rotated

    print(f"Testing Yaw (in radians): {yaw_radians}")
    print("GPS coordinates (X, Y):", round(x_gps, 2), round(y_gps, 2))
    print("Base link coordinates (X, Y):", round(x_base_link, 2), round(y_base_link, 2))
    print("------------------------------------------------")
