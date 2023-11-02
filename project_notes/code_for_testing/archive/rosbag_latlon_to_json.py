#!/usr/bin/env python
'''
Script that reads the lat, lon data from the /fix topic in a rosbag file and outputs just the lat, lon data into a .json file.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_latlon_to_json.py
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

# Filter the mission_polygon list to include only points that are at least 0.5 meters apart
def filter_points(points, min_distance):
    filtered_points = []
    prev_point = None
    for point in points:
        if prev_point is None or haversine(prev_point['lat'], prev_point['lng'], point['lat'], point['lng']) >= min_distance:
            filtered_points.append(point)
            prev_point = point
    return filtered_points    

# Path to the rosbag file
bagfile_path = '/home/tractor/bagfiles/'
filename = '2023-11-01-14-51-56'
filetype = '.bag'
rosbag_path = bagfile_path + filename + filetype

# Initialize an empty list to hold the latitude and longitude data
mission_polygon = []

# Open the rosbag file
print(f"Opening the rosbag and extracting all lat/lon records")
with rosbag.Bag(rosbag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        if topic == '/fix':
            # Extract latitude and longitude from the message
            lat = msg.latitude
            lon = msg.longitude
            # Append the extracted data to the mission_polygon list
            mission_polygon.append({'lat': lat, 'lng': lon})
            #print(f"Record written: Latitude: {lat}, Longitude: {lon}")

record_count = len(mission_polygon)
print(f"Number of lat/lon records in rosbag file: {record_count}")
# Replace the existing mission_polygon list creation with the filtered version
mission_polygon_filtered = filter_points(mission_polygon, 3)
record_count = len(mission_polygon_filtered)
print(f"Number of lat/lon records in filtered file: {record_count}")


# Load the structure of the JSON file from the provided example
example_json_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step1.json'
with open(example_json_file, 'r') as file:
    example_json_data = json.load(file)

# Replace the missionPolygon data with the data extracted from the rosbag
example_json_data['missionPolygon'] = mission_polygon

# Save the new data to a JSON file
output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_from_rosbag.json'
with open(output_file_path, 'w') as file:
    json.dump(example_json_data, file, indent=4)

print(f"Data has been converted to .json file and saved to {output_file_path}")
