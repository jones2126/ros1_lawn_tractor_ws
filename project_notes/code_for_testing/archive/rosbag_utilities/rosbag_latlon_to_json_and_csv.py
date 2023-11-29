#!/usr/bin/env python
'''
Script that reads the lat, lon data from the /fix topic in a rosbag file and outputs just the lat, lon data into a .json file.

$ python /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_latlon_to_json_and_csv.py
'''

import rosbag
import json
import math

# Haversine formula to calculate the distance between two points on the Earth
def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000
    # Convert latitude and longitude from degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Calculate the distance
    a = math.sin(delta_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

# Filter the mission_polygon list to include only points that are at least {min_distance} apart (in meters)
def filter_points(points, min_distance):
    filtered_points = []
    prev_point = None
    for point in points:
        if prev_point is None or haversine(prev_point['lat'], prev_point['lng'], point['lat'], point['lng']) >= min_distance:
            filtered_points.append(point)
            prev_point = point
    return filtered_points    

# Collins_Dr_62_Site_01_run2_2023-11-01-14-51-56.bag
bagfile_path = '/home/tractor/bagfiles/'
#rosbag_filename = '2023-11-01-14-02-20'
rosbag_filename = 'Collins_Dr_62_Site_01_run2_2023-11-01-14-51-56'
filetype = '.bag'
rosbag_path = bagfile_path + rosbag_filename + filetype


mission_polygon = []
print(f"Opening the rosbag and extracting all lat/lon records")
mission_polygon_csv = []
start_time = None 
with rosbag.Bag(rosbag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        if start_time is None:
            start_time = t.to_sec()
        time_delta = t.to_sec() - start_time   # Calculate the time delta
        lat = msg.latitude  # Extract latitude and longitude from the message
        lon = msg.longitude
        timestamp_raw = t  # Get the raw timestamp (secs and nsecs)
        timestamp_seconds = t.to_sec()  # Convert timestamp to seconds
        mission_polygon.append({'lat': lat, 'lng': lon})  # to create a .json file later
        # Create a list to make a .csv file later
        mission_polygon_csv.append({
            'time_delta': time_delta,
            'timestamp_seconds': timestamp_seconds,
            'timestamp_raw': str(timestamp_raw),  # Convert the raw timestamp to string for JSON serialization
            'lat': lat, 
            'lng': lon
        })
        # print(f"Record written: Time: {timestamp_seconds}, Raw Time: {str(timestamp_raw)}, Latitude: {lat}, Longitude: {lon}")
record_count = len(mission_polygon)
print(f"Number of lat/lon records in rosbag file: {record_count}")

mission_polygon_filtered = filter_points(mission_polygon, 3)
record_count = len(mission_polygon_filtered)
print(f"Number of lat/lon records in filtered file: {record_count}")


# Create a JSON file from an example form
pathfile_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/'
json_filename = 'collins_dr_62_A_step1'
filetype = '.json'
example_json_file = pathfile_path + json_filename + filetype
#example_json_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step1.json'
with open(example_json_file, 'r') as file:
    example_json_data = json.load(file)
example_json_data['missionPolygon'] = mission_polygon  # Replace the missionPolygon data with the data extracted from the rosbag
output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/collins_dr_62_A_from_rosbag.json'
with open(output_file_path, 'w') as file:
    json.dump(example_json_data, file, indent=4)
print(f"Data has been converted to .json file and saved to {output_file_path}")


import csv
print(f"Creating a .csv file of lat, lon data")
csv_filename = 'collins_dr_62_A_from_rosbag_run2_ver2'
filetype = '.csv'
csv_file_path = pathfile_path + csv_filename + filetype
#csv_header = ['timestamp_seconds', 'timestamp_raw', 'lat', 'lng']
csv_header = ['time_delta', 'timestamp_seconds', 'timestamp_raw', 'lat', 'lng']
with open(csv_file_path, 'w', newline='') as csv_file:
    writer = csv.DictWriter(csv_file, fieldnames=csv_header)
    writer.writeheader() # Write the header
    for record in mission_polygon_csv:  # Write the data
        writer.writerow(record)
print(f"CSV file '{csv_file_path}' has been created with {len(mission_polygon_csv)} records.")