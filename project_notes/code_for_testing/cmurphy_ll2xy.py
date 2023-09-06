#!/usr/bin/env python3
'''
A conversion routine between latitude-longitude and x-y coordinates. It is based on the Mercator projection for conversion.

The code appears in the paper, "Rectilinear Coordinate Frames for Deep Sea Navigation" by:
Chris Murphy, Deep Submergence Laboratory referenced here: https://wiki.nps.edu/display/RC/Local+Coordinate+Frames

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/cmurphy_ll2xy.py

'''

import math

def latlon2xy(lat, long, lat0, lon0):
    # Returns a tuple: (x,y) where...
    #     x is Easting 
    #     y is Northing 
    # which I believe follows the ROS REP103 ENU convention    
    x = (long-lon0) * mdeglon(lat0)
    y = (lat-lat0) * mdeglat(lat0)
    return x, y

def xy2latlon(x, y, lat0, lon0):
    lon = x/mdeglon(lat0) + lon0;
    lat = y/mdeglat(lat0) + lat0;
    return lat, lon    

def mdeglon(latitude_in_decimal_degrees):
    # Provides meters-per-degree longitude at a given latitude
    latrad = lat*2.0*pi/360.0  # convert latitude to radians
    meters_per_degree_longitude = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad) + 0.12 * cos(5.0*latrad)
    return meters_per_degree_longitude

def mdeglat(latitude_in_decimal_degrees):
    # Provides meters-per-degree latitude at a given latitude
    latrad = lat*2.0*pi/360.0  # convert latitude to radians
    meters_per_degree_latitude = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad) - 0.002 * cos(6.0*latrad)
    return meters_per_degree_latitude
