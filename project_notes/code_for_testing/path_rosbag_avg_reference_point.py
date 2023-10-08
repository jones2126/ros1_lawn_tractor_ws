#!/usr/bin/env python
'''
Script that reads the lat lon data from the bagfile and averages them.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_rosbag_avg_reference_point.py

'''
import math
import rosbag
import geonav_transform.geonav_conversions as gc
import utm

# Set the origin latitude and longitude
origin_lat = 40.34534080
origin_lon = -80.12894600

# Get UTM zone for the origin
origin_utm_zone = utm.from_latlon(origin_lat, origin_lon)[2]

# Set the bag path
bag_path = '/home/tractor/bagfiles/2023-09-22-10-51-43Stationary_at_reference_point.bag'

# Initialize variables
lat_sum = 0
lon_sum = 0
count = 0

def adjust_heading_to_enu(adjusted_heading_rad):
    """
    Adjusts the heading to the specified ENU convention.  The input can be 2*Pi(), the output will be +/- Pi()
    """
    if adjusted_heading_rad > math.pi:
        heading_enu = adjusted_heading_rad - 2 * math.pi
    else:
        heading_enu = adjusted_heading_rad
    
    return heading_enu

def heading_between_two_points(lat1, lon1, lat2, lon2):
    """
    Compute the heading (in radians) between two points using the ENU convention based on a flat Earth approximation.
    """
    # Convert degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Approximate the local Cartesian coordinates using flat Earth approximation
    R_earth = 6378137  # radius of Earth in meters using WGS-84 at the equator
    x_diff = R_earth * (lon2 - lon1) * math.cos(lat1)
    y_diff = R_earth * (lat2 - lat1)
    
    # Compute the heading based on the difference in coordinates
    raw_heading_rad = math.atan2(y_diff, x_diff)
    
    # Adjust to the ENU convention
    adjusted_heading_rad = (raw_heading_rad + 2 * math.pi) % (2 * math.pi)
    
    return adjusted_heading_rad    

# Open the bag file
with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        if msg.status.status == 2:  # Check if the fix status is 2
            lat_sum += msg.latitude
            lon_sum += msg.longitude
            count += 1

# Compute the average latitude and longitude
avg_lat = lat_sum / count if count else 0
avg_lon = lon_sum / count if count else 0
averaged_position = (avg_lat, avg_lon)

print("Averaged Position:", averaged_position)

# Compute distance and heading between origin and averaged position
# Convert origin to local x, y
origin_x, origin_y = gc.ll2xy(origin_lat, origin_lon, origin_lat, origin_lon)

# Convert averaged position to local x, y
x, y = gc.ll2xy(avg_lat, avg_lon, origin_lat, origin_lon)
print("Local Coordinates of Averaged Position: X =", x, ", Y =", y)

# Calculate distance in inches
distance = ((x - origin_x) ** 2 + (y - origin_y) ** 2) ** 0.5 * 39.37  # Convert meters to inches
print("Distance (inches):", distance)

# Calculate heading in radians
heading = adjust_heading_to_enu(heading_between_two_points(origin_lat, origin_lon, avg_lat, avg_lon))
print("Heading (radians):", heading)
