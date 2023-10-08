#!/usr/bin/env python


'''
Used to convert known lat lon positions to x, y coordinates based on a known origin lat / lon.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_convert_ll2xy_v3.py

'''

from geonav_transform.geonav_conversions import ll2xy
from math import cos, pi, radians, sqrt

def cmurphy_ll2xy(lat, long, lat0, lon0):
    # This code appears in the paper, "Rectilinear Coordinate Frames for Deep Sea Navigation" by:
    # Chris Murphy, Deep Submergence Laboratory referenced here: https://wiki.nps.edu/display/RC/Local+Coordinate+Frames    
    # It returns a tuple: (x,y) where... x is Easting and y is Northing 
    x = (long-lon0) * mdeglon(lat0)
    y = (lat-lat0) * mdeglat(lat0)
    return x, y

def mdeglon(latitude_in_decimal_degrees):
    # Provides meters-per-degree longitude at a given latitude
    latrad = latitude_in_decimal_degrees*2.0*pi/360.0
    meters_per_degree_longitude = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad) + 0.12 * cos(5.0*latrad)
    return meters_per_degree_longitude

def mdeglat(latitude_in_decimal_degrees):
    # Provides meters-per-degree latitude at a given latitude
    latrad = latitude_in_decimal_degrees*2.0*pi/360.0
    meters_per_degree_latitude = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad) - 0.002 * cos(6.0*latrad)
    return meters_per_degree_latitude

def latlon_to_xy(lat, lon, origin_lat, origin_lon):
    # Convert latitude and longitude to x and y coordinates using the provided origin point with a flat Earth approximation.
    R_earth = 6378137  # Radius of Earth in meters (WGS-84)
    lat_rad, lon_rad = radians(lat), radians(lon)  # Convert degrees to radians
    origin_lat_rad, origin_lon_rad = radians(origin_lat), radians(origin_lon)
    x_diff = R_earth * (lon_rad - origin_lon_rad) * cos(origin_lat_rad)  # Compute x and y differences from the origin
    y_diff = R_earth * (lat_rad - origin_lat_rad)
    return x_diff, y_diff

def calculate_distance(x1, y1, x2, y2):
    #Calculate the distance between two points using the Euclidean formula
    x_diff = x2 - x1
    y_diff = y2 - y1
    x_diff_squared = x_diff**2
    y_diff_squared = y_diff**2
    distance = sqrt(x_diff_squared + y_diff_squared)
    return distance        

# Displaying the latitude, longitude and the corresponding x, y coordinates
print("Lat, Lon and corresponding X, Y coordinates:")
print("=============================================")

previous_x1, previous_y1 = None, None
previous_x2, previous_y2 = None, None

origin_lat = 40.34534080; origin_lon = -80.12894600  # represents the starting point of my tractor inside the garage - averaged on 20230904

# I averaged these positions for three minutes with RTK GPS fix and marked the locations on the ground
# averaged_positions = [
#     (40.345300971941796, -80.1289413912553),
#     (40.34530662627222, -80.12887740598163),
#     (40.345268310965714, -80.12887049172689),
#     (40.3452617478205, -80.12895260716566)
# ]

averaged_positions = [(40.34530215911272, -80.12893947576222), (40.345314858326475, -80.12888316346998), (40.34531428439999, -80.12888852100001), (40.34531285833334, -80.12892637793334)]


for lat, lon in averaged_positions:
    x1, y1 = latlon_to_xy(lat, lon, origin_lat, origin_lon)
    x2, y2 = ll2xy(lat, lon, origin_lat, origin_lon)
    x3, y3 = cmurphy_ll2xy(lat, lon, origin_lat, origin_lon)
    
    print(f"Lat: {lat:.8f}, Lon: {lon:.8f}")
    print(f"latlon_to_xy => X: {x1:.3f}, Y: {y1:.3f}")
    print(f"geonav.ll2xy  => X: {x2:.3f}, Y: {y2:.3f}")
    print(f"cmurphy_ll2xy     => X: {x3:.3f}, Y: {y3:.3f}")
    
    if previous_x1 is not None:
        dist1 = calculate_distance(previous_x1, previous_y1, x1, y1)
        dist2 = calculate_distance(previous_x2, previous_y2, x2, y2)
        dist3 = calculate_distance(previous_x3, previous_y3, x3, y3)
        
        print(f"Distance between this point and previous using latlon_to_xy: {dist1:.3f} meters ({dist1 * 39.3701:.3f} inches)")
        print(f"Distance between this point and previous using geonav.ll2xy: {dist2:.3f} meters ({dist2 * 39.3701:.3f} inches)")
        print(f"Distance between this point and previous using cmurphy_ll2xy: {dist3:.3f} meters ({dist3 * 39.3701:.3f} inches)")
    
    print("-------------------------------")
    
    previous_x1, previous_y1 = x1, y1
    previous_x2, previous_y2 = x2, y2
    previous_x3, previous_y3 = x3, y3
