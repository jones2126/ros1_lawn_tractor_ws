#!/usr/bin/env python

'''
Script that reads the lat lon data from the bagfile mentioned above 
betweenstarting and ending point in seconds, averages those positions and then calculates
distance and heading between the points.

$python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_rosbag_avg_pts_with_dist.py

'''

import math

def compute_bearing(lat1, lon1, lat2, lon2):
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    
    y = math.sin(delta_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
    
    return math.atan2(y, x)



import rosbag
import rospy
import geonav_transform.geonav_conversions as gc
import utm

origin_lat = 40.3452982
origin_lon = -80.1288761


# Get UTM zone for the origin
origin_utm_zone = utm.from_latlon(origin_lat, origin_lon)[2]

bag_path = '/home/tractor/bagfiles/2023-09-03-17-26-42.bag'
bag = rosbag.Bag(bag_path)

first_timestamp = None
outside_utm_count = 0
total_messages = 0

# Define time intervals and data structure to hold accumulated data
time_intervals = [(7, 276), (328, 546), (594, 788), (826, 1021)]
interval_data = [{'lat_sum': 0, 'lon_sum': 0, 'count': 0} for _ in time_intervals]

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
    heading = compute_bearing(prev_lat, prev_lon, lat, lon)
    headings.append(heading)
    
    # Update previous point for next iteration
    prev_x, prev_y = x, y

print("Distances (inches):", distances)
print("Headings (radians):", headings)
print(f"\nTotal messages: {total_messages}")
print(f"Messages outside origin's UTM zone: {outside_utm_count}")