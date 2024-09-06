#!/usr/bin/env python3

'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/ReviewLatLonforGforceAnomalies_v2.py

This script does the following:

Reads the rosbag file and extracts GPS data only for the time period between 287 and 376 seconds.
Calculates the distance, bearing, and speed between consecutive points.
Calculates the acceleration between consecutive points.
Identifies anomalies based on a threshold acceleration (currently set to 0.5 m/s^2, but you can adjust this).
Prints detailed information for the first five anomalies.

The output will show:

The timestamp (in seconds since the start of the bag file)
The 'from' and 'to' latitude and longitude
The calculated distance between points
The bearing (direction of travel)
The speed between points
The calculated acceleration

bag_file = '/home/tractor/bagfiles/2024-08-27-14-33-33.bag'
'''
import rosbag
from rosbag import Bag
import numpy as np
from geopy import distance
import math
import csv
from collections import defaultdict

def calculate_bearing(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    initial_bearing = math.atan2(y, x)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

bag_file = '/home/tractor/bagfiles/2024-08-27-14-33-33.bag'
csv_file = 'tractor_data_260_380.csv'

data = defaultdict(dict)
start_time = None
odom_data = {}

with Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/fix', '/odom']):
        if start_time is None:
            start_time = t.to_sec()
        relative_time = t.to_sec() - start_time
        
        if 260 <= relative_time <= 380:
            if topic == '/fix':
                data[relative_time].update({
                    'timestamp': relative_time,
                    'latitude': msg.latitude,
                    'longitude': msg.longitude,
                    'rtk_status': msg.status.status  # Adding RTK status
                })
            elif topic == '/odom':
                odom_data[relative_time] = msg.twist.twist.linear.x

# Merge GPS and odometry data
for t in data:
    nearest_odom_time = min(odom_data.keys(), key=lambda x: abs(x - t))
    data[t]['odom_linear_x'] = odom_data[nearest_odom_time]

# Sort data by timestamp
sorted_data = sorted(data.values(), key=lambda x: x['timestamp'])

# Calculate distances, bearings, speeds, and accelerations
for i in range(len(sorted_data)):
    if i == 0:
        sorted_data[i]['distance'] = 0
        sorted_data[i]['bearing'] = 0
        sorted_data[i]['speed'] = 0
        sorted_data[i]['acceleration'] = 0
        continue
    
    prev = sorted_data[i-1]
    curr = sorted_data[i]
    
    dist = distance.distance(
        (prev['latitude'], prev['longitude']),
        (curr['latitude'], curr['longitude'])
    ).meters
    
    bearing = calculate_bearing(
        prev['latitude'], prev['longitude'],
        curr['latitude'], curr['longitude']
    )
    
    time_diff = curr['timestamp'] - prev['timestamp']
    speed = dist / time_diff if time_diff > 0 else 0
    
    curr['distance'] = dist
    curr['bearing'] = bearing
    curr['speed'] = speed

    speed_diff = curr['speed'] - prev['speed']
    acceleration = speed_diff / time_diff if time_diff > 0 else 0
    curr['acceleration'] = acceleration

# Write data to CSV file
with open(csv_file, 'w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=['timestamp', 'latitude', 'longitude', 'rtk_status', 'distance', 'bearing', 'speed', 'acceleration', 'odom_linear_x'])
    writer.writeheader()
    for row in sorted_data:
        writer.writerow(row)

total_points = len(sorted_data)
time_period = sorted_data[-1]['timestamp'] - sorted_data[0]['timestamp']
average_hz = total_points / time_period

print(f"Data written to {csv_file}")
print(f"Total data points: {total_points}")
print(f"Time period: {time_period:.2f} seconds")
print(f"Average frequency: {average_hz:.2f} Hz")