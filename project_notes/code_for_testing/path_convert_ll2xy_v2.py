#!/usr/bin/env python


'''
Used to convert known lat lon positions to x, y coordinates based on a known origin lat / lon.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_convert_ll2xy_v2.py

'''

import math
from geonav_transform.geonav_conversions import ll2xy

def latlon_to_xy(lat, lon, origin_lat, origin_lon):
    """
    Convert latitude and longitude to x and y coordinates using the provided origin point.
    Uses a flat Earth approximation.
    """
    R_earth = 6378137  # Radius of Earth in meters (WGS-84)
    
    # Convert degrees to radians
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    origin_lat_rad, origin_lon_rad = math.radians(origin_lat), math.radians(origin_lon)
    
    # Compute x and y differences from the origin
    x_diff = R_earth * (lon_rad - origin_lon_rad) * math.cos(origin_lat_rad)
    y_diff = R_earth * (lat_rad - origin_lat_rad)
    
    return x_diff, y_diff

def calculate_distance(x1, y1, x2, y2):
    """Calculate the distance between two points"""
    x_diff = x2 - x1
    y_diff = y2 - y1
    x_diff_squared = x_diff**2
    y_diff_squared = y_diff**2
    distance = math.sqrt(x_diff_squared + y_diff_squared)  # Compute the distance using the Euclidean formula
    return distance   

origin_lat = 40.34534080; origin_lon = -80.12894600

# Given averaged_positions list
averaged_positions = [
    (40.345300971941796, -80.1289413912553),
    (40.34530662627222, -80.12887740598163),
    (40.345268310965714, -80.12887049172689),
    (40.3452617478205, -80.12895260716566)
]

# Displaying the latitude, longitude and the corresponding x, y coordinates
print("Lat, Lon and corresponding X, Y coordinates:")
print("=============================================")

previous_x1, previous_y1 = None, None
previous_x2, previous_y2 = None, None

for lat, lon in averaged_positions:
    x1, y1 = latlon_to_xy(lat, lon, origin_lat, origin_lon)
    x2, y2 = ll2xy(lat, lon, origin_lat, origin_lon)
    
    print(f"Lat: {lat:.8f}, Lon: {lon:.8f}")
    print(f"latlon_to_xy => X: {x1:.3f}, Y: {y1:.3f}")
    print(f"geonav.ll2xy  => X: {x2:.3f}, Y: {y2:.3f}")

    print("-------------------------------")

    if previous_x1 is not None:
        dist1 = calculate_distance(previous_x1, previous_y1, x1, y1)
        dist2 = calculate_distance(previous_x2, previous_y2, x2, y2)

        print(f"Distance between this point and previous using latlon_to_xy: {dist1:.3f} meters ({dist1 * 39.3701:.1f} inches)")
        print(f"Distance between this point and previous using geonav.ll2xy: {dist2:.3f} meters ({dist2 * 39.3701:.1f} inches)")


        print("-------------------------------")
    
    previous_x1, previous_y1 = x1, y1
    previous_x2, previous_y2 = x2, y2