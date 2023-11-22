#!/usr/bin/env python3
'''
Script that calculates a path based on 4 corner points and moves inward.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_generator_using_4_pts.py

'''
#from path_generator_utilities import calculate_distance, calculate_polygon_area, calculate_total_length
from path_generator_utilities import create_polygon
#create_polygon
# Let's write a test program for the create_polygon function with the given parameters
origin_test = (0, 0)  # Origin point
sides_test = 4  # Number of sides
area_test = 81  # Area in square meters
# 5.54909
# Since the function has already been defined in a previous cell, we can directly call it
test_polygon_points, side_length = create_polygon(origin_test, sides_test, area_test)
print(side_length)
print(test_polygon_points)
