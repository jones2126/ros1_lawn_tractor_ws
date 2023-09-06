#!/usr/bin/env python

'''
Script that reads the lat lon data from the bagfile mentioned above 
between a starting and ending point in seconds, converts those positions 
to x and y coordinates, and then plots them using pyplot.

$python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_rosbag_avg_pts.py

'''

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
print(f"\nTotal messages: {total_messages}")
print(f"Messages outside origin's UTM zone: {outside_utm_count}")
