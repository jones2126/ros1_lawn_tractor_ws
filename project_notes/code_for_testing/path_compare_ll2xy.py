#!/usr/bin/env python3
'''
Trying to compare the differences in results using code from the paper, "Rectilinear Coordinate Frames for Deep Sea Navigation"
by Chris Murphy, Deep Submergence Laboratory referenced here: https://wiki.nps.edu/display/RC/Local+Coordinate+Frames

Additional comparison to perform: https://docs.ros.org/en/kinetic/api/geonav_transform/html/_modules/alvinxy/alvinxy.html

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_compare_ll2xy.py

'''
from math import pi, cos, sqrt
import geonav_transform.geonav_conversions as gc

class CMurphy:
    @staticmethod  # Using @staticmethod decorator so the function operates using only the parameters provided 
    def latlon2xy(lat, long, lat0, lon0):
        # Returns a tuple: (x,y) where...
        #     x is Easting 
        #     y is Northing 
        # which I believe follows the ROS REP103 ENU convention  
        x = (long-lon0) * CMurphy.mdeglon(lat0)
        y = (lat-lat0) * CMurphy.mdeglat(lat0)
        return x, y

    @staticmethod
    def xy2latlon(x, y, lat0, lon0):
        lon = x/CMurphy.mdeglon(lat0) + lon0;
        lat = y/CMurphy.mdeglat(lat0) + lat0;
        return lat, lon    

    @staticmethod
    def mdeglon(latitude_in_decimal_degrees):
        # Provides meters-per-degree longitude at a given latitude
        latrad = latitude_in_decimal_degrees*2.0*pi/360.0  # convert latitude to radians
        meters_per_degree_longitude = 111415.13 * cos(latrad) - 94.55 * cos(3.0*latrad) + 0.12 * cos(5.0*latrad)
        return meters_per_degree_longitude
    
    @staticmethod
    def mdeglat(latitude_in_decimal_degrees):
        # Provides meters-per-degree latitude at a given latitude
        latrad = latitude_in_decimal_degrees*2.0*pi/360.0  # convert latitude to radians
        meters_per_degree_latitude = 111132.09 - 566.05 * cos(2.0*latrad) + 1.20 * cos(4.0*latrad) - 0.002 * cos(6.0*latrad)
        return meters_per_degree_latitude


# Comparison code:
origin_lat = 40.3452982
origin_lon = -80.1288761

# Compute change in longitude for a move of 5 meters due East
delta_lon = 5 / CMurphy.mdeglon(origin_lat)
new_lon = origin_lon + delta_lon

# Compute change in latitude for a move of 1 meter due North
delta_lat = 1 / CMurphy.mdeglat(origin_lat)
new_lat = origin_lat + delta_lat

x_custom, y_custom = CMurphy.latlon2xy(new_lat, new_lon, origin_lat, origin_lon)
x_geonav, y_geonav = gc.ll2xy(new_lat, new_lon, origin_lat, origin_lon)

print(f"c_murphy Implementation: x = {x_custom}, y = {y_custom}")
print(f"Geonav Implementation: x = {x_geonav}, y = {y_geonav}")


origin_lat = 40.3452982
origin_lon = -80.1288761
#known_lat_p1 = 40.345307045
#known_lon_p1 = -80.12885537666666

known_lat_p1 = 40.34530831166666
known_lon_p1 = -80.12885524666666


# Convert origin and known position to x,y using Custom approach
# coordinates using the origin itself as the reference point, the resulting x,y coordinates should be (0, 0) 
x_origin_custom, y_origin_custom = CMurphy.latlon2xy(origin_lat, origin_lon, origin_lat, origin_lon)
x_known_custom, y_known_custom = CMurphy.latlon2xy(known_lat_p1, known_lon_p1, origin_lat, origin_lon)

# Compute distance using Custom approach
distance_custom = sqrt((x_known_custom - x_origin_custom)**2 + (y_known_custom - y_origin_custom)**2)

# Convert origin and known position to x,y using geonav approach
x_origin_geonav, y_origin_geonav = gc.ll2xy(origin_lat, origin_lon, origin_lat, origin_lon)
x_known_geonav, y_known_geonav = gc.ll2xy(known_lat_p1, known_lon_p1, origin_lat, origin_lon)

# Compute distance using geonav approach
distance_geonav = sqrt((x_known_geonav - x_origin_geonav)**2 + (y_known_geonav - y_origin_geonav)**2)

print(f"Distance using c_murphy Implementation: {distance_custom} meters")
print(f"Distance using Geonav Implementation: {distance_geonav} meters")

# Displacement using Custom approach
delta_x_custom = x_known_custom - x_origin_custom
delta_y_custom = y_known_custom - y_origin_custom

# Displacement using geonav approach
delta_x_geonav = x_known_geonav - x_origin_geonav
delta_y_geonav = y_known_geonav - y_origin_geonav

print(f"Displacement in X using c_murphy Implementation: {delta_x_custom} meters")
print(f"Displacement in Y using c_murphy Implementation: {delta_y_custom} meters")
print(f"Displacement in X using Geonav Implementation: {delta_x_geonav} meters")
print(f"Displacement in Y using Geonav Implementation: {delta_y_geonav} meters")
