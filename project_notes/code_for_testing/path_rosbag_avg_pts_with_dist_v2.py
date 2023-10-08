#!/usr/bin/env python


'''
Script that reads the lat lon data from the bagfile mentioned above 
between a starting and ending point in seconds, converts those positions 
to x and y coordinates, and then plots them using pyplot.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_rosbag_avg_pts_with_dist_v2.py

'''

import math
import rosbag
import rospy
import geonav_transform.geonav_conversions as gc
import utm

origin_lat = 40.34534080; origin_lon = -80.12894600  # represents the starting point of my tractor inside the garage - averaged on 20230904

# Get UTM zone for the origin
origin_utm_zone = utm.from_latlon(origin_lat, origin_lon)[2]

#bag_path = '/home/tractor/bagfiles/2023-09-03-17-26-42.bag'  # used to calculate origin
#bag_path = '/home/tractor/bagfiles/2023-09-06-11-57-26.bag'  # used to reference_point
bag_path = '/home/tractor/bagfiles/2023-09-22-10-51-43Stationary_at_reference_point.bag'
bag = rosbag.Bag(bag_path)

first_timestamp = None
outside_utm_count = 0
total_messages = 0

# Define time intervals and data structure to hold accumulated data
time_intervals = [(1, 37), (40, 620), (621, 1997), (2010, 2195)]
interval_data = [{'lat_sum': 0, 'lon_sum': 0, 'count': 0} for _ in time_intervals]

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

for topic, msg, t in bag.read_messages(topics=['/fix']):
    timestamp_seconds = t.to_sec()

    if first_timestamp is None:
        first_timestamp = timestamp_seconds

    relative_time = timestamp_seconds - first_timestamp

    for idx, (start, end) in enumerate(time_intervals):
        if start <= relative_time <= end:
            lat = msg.latitude
            lon = msg.longitude
            interval_data[idx]['lat_sum'] += lat
            interval_data[idx]['lon_sum'] += lon
            interval_data[idx]['count'] += 1

            # Get UTM zone for the current point
            current_utm_zone = utm.from_latlon(lat, lon)[2]

            # Check if UTM zones are different
            if current_utm_zone != origin_utm_zone:
                outside_utm_count += 1
                print(f"relative_time: {relative_time} Lat: {lat}, Lon: {lon} - UTM Zone: {current_utm_zone} (Different from origin's UTM Zone: {origin_utm_zone})")
            
            total_messages += 1

bag.close()

# Compute average lat/lon for each interval after processing the bag
averaged_positions = []
for data in interval_data:
    avg_lat = data['lat_sum'] / data['count'] if data['count'] else 0
    avg_lon = data['lon_sum'] / data['count'] if data['count'] else 0
    averaged_positions.append((avg_lat, avg_lon))

print("Averaged Positions:", averaged_positions)

# Compute distance and heading between points

# Convert origin to local x, y
origin_x, origin_y = gc.ll2xy(origin_lat, origin_lon, origin_lat, origin_lon)

# Initialize previous point as origin

# Initialize previous latitude and longitude as origin
prev_lat, prev_lon = origin_lat, origin_lon
prev_x, prev_y = origin_x, origin_y

# Store distances and headings
distances = []
headings = []

for lat, lon in averaged_positions:
    # Convert current lat, lon to local x, y
    x, y = gc.ll2xy(lat, lon, origin_lat, origin_lon)
    
    # Calculate distance in inches
    distance = ((x - prev_x)**2 + (y - prev_y)**2)**0.5 * 39.37  # Convert meters to inches
    distances.append(distance)
    
    # Calculate heading in radians
    heading = adjust_heading_to_enu(heading_between_two_points(prev_lat, prev_lon, lat, lon))
    print(prev_lat, prev_lon, lat, lon, heading)
    headings.append(heading)
    
    # Update previous point for next iteration
    prev_lat = lat; prev_lon = lon # used for heading
    prev_x = x; prev_y = y  # used for distance

print("Distances (inches):", distances)
print("Headings (radians):", headings)
print(f"\nTotal messages: {total_messages}")
print(f"Messages outside origin's UTM zone: {outside_utm_count}")