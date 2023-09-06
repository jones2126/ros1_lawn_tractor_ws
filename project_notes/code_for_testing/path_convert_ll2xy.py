#!/usr/bin/env python


'''
Used to convert known lat lon positions to x, y coordinates based on a known origin lat / lon.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_convert_ll2xy.py

'''

import math

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

for lat, lon in averaged_positions:
    x, y = latlon_to_xy(lat, lon, origin_lat, origin_lon)
    print(f"Lat: {lat}, Lon: {lon} => X: {x}, Y: {y}")
