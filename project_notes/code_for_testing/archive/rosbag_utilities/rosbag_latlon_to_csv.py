#!/usr/bin/env python

'''
Script that reads the lat, lon data from the /fix topic in a rosbag file and outputs just the lat, lon data into a .csv file.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_latlon_to_csv.py
'''

import rosbag
import csv
import math

# Haversine formula
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi, delta_lambda = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def filter_points(points, min_distance):
    filtered_points = []
    prev_point = None
    for point in points:
        if prev_point is None or haversine(prev_point['lat'], prev_point['lng'], point['lat'], point['lng']) >= min_distance:
            filtered_points.append(point)
            prev_point = point
    return filtered_points

# Paths
bagfile_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
rosbag_filename = '62Collins_perimiter_1_2024-04-17-18-34-40.bag'
csv_filename = 'collins_dr_62_A_from_rosbag_step1_20240513_2.csv'
gap_between_pts = 1

# Data processing
mission_data = []
odom_data = {}
start_time = None 
with rosbag.Bag(bagfile_path + rosbag_filename, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/fix', '/odom']):
        if start_time is None:
            start_time = t.to_sec()        
        if topic == '/fix':
            time_delta = t.to_sec() - start_time   # Correctly calculate time delta before the dictionary
            mission_data.append({
                'time_delta': time_delta,  # Use a colon to correctly assign time_delta in the dictionary
                'timestamp_seconds': t.to_sec(),
                'timestamp_raw': str(t),
                'lat': msg.latitude,
                'lng': msg.longitude,
                'pose_x': None,  # placeholder for pose data
                'pose_y': None
            })
        elif topic == '/odom':
            odom_data[t.to_sec()] = {
                'pose_x': msg.pose.pose.position.x,
                'pose_y': msg.pose.pose.position.y
            }

# Synchronize GPS and Odom data based on closest timestamp
'''
This lambda function calculates the absolute difference between the odometry timestamp t and the GPS 
timestamp for each odometry data point, and min uses this to find the timestamp where this difference is the smallest.
'''
for record in mission_data:
    closest_time = min(odom_data.keys(), key=lambda t: abs(t - record['timestamp_seconds']))
    record['pose_x'] = odom_data[closest_time]['pose_x']
    record['pose_y'] = odom_data[closest_time]['pose_y']

record_count = len(mission_data)
print(f"Number of lat/lon records in raw rosbag file: {record_count}")

# Filter the mission data
mission_data = filter_points(mission_data, gap_between_pts)

record_count = len(mission_data)
print(f"Number of lat/lon records in trimmed rosbag file: {record_count}")

# Writing to CSV
with open(bagfile_path + csv_filename, 'w', newline='') as csv_file:
    fieldnames = ['time_delta', 'timestamp_seconds', 'timestamp_raw', 'lat', 'lng', 'pose_x', 'pose_y']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    for record in mission_data:
        writer.writerow(record)

print(f"CSV file '{csv_filename}' has been created with {len(mission_data)} records.")
