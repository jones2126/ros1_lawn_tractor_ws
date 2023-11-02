#!/usr/bin/env python
'''
Script that reads the lat, lon data from the /fix topic and x, y positions from 
/odom/pose/pose/position topic in a ROS bag file, between a starting and ending 
point in seconds, converts those positions to x, y coordinates and then plots them 
using pyplot.

$python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_get_latlon_odom.py
'''
import matplotlib.pyplot as plt
import math
import csv

# Function to calculate Euclidean distance between two points
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

import rosbag
import rospy
import geonav_transform.geonav_conversions as gc
import utm

# 62 Collins Dr
origin_lat = 40.48528569166667
origin_lon = -80.33262521333333

# Get UTM zone for the origin
origin_utm_zone = utm.from_latlon(origin_lat, origin_lon)[2]

# Path to the ROS bag file
#bag_path = '/home/tractor/bagfiles/2023-11-01-14-51-56.bag'
bag_path = '/home/tractor/bagfiles/2023-11-01-14-02-20.bag'
bag = rosbag.Bag(bag_path)

# Time range for reading messages
start_time = 100
end_time = 690

first_timestamp = None
outside_utm_count = 0
total_fix_messages = 0
total_odom_messages = 0

# Containers to store lat, lon, x, y data
lat_data = []
lon_data = []
x_data = []
y_data = []
filtered_x_data = []
filtered_y_data = []

for topic, msg, t in bag.read_messages(topics=['/fix', '/odom']):
    timestamp_seconds = t.to_sec()

    if first_timestamp is None:
        first_timestamp = timestamp_seconds

    relative_time = timestamp_seconds - first_timestamp

    if start_time <= relative_time <= end_time:
        if topic == '/fix':
            lat = msg.latitude
            lon = msg.longitude

            # Convert to UTM and store data
            utm_coords = utm.from_latlon(lat, lon)
            lat_data.append(utm_coords[0])
            lon_data.append(utm_coords[1])

            # Get UTM zone for the current point
            current_utm_zone = utm_coords[2]

            # Check if UTM zones are different
            if current_utm_zone != origin_utm_zone:
                outside_utm_count += 1
                print(f"relative_time: {relative_time} Lat: {lat}, Lon: {lon} - UTM Zone: {current_utm_zone} (Different from origin's UTM Zone: {origin_utm_zone})")
            
            total_fix_messages += 1
        
        # elif topic == '/odom/pose/pose/position':
        #     x = msg.x
        #     y = msg.y

        #     # Store x, y data
        #     x_data.append(x)
        #     y_data.append(y)
        #     total_odom_messages += 1

    if topic == '/odom':
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Store x, y data
        x_data.append(x)
        y_data.append(y)

        print(f"X: {x}, Y: {y}")

        # Filter points
        if not filtered_x_data or euclidean_distance(x, y, filtered_x_data[-1], filtered_y_data[-1]) >= 0.5:
            filtered_x_data.append(x)
            filtered_y_data.append(y)
        

bag.close()

# Summary
print(f"\nTotal /fix messages: {total_fix_messages}")
print(f"Messages outside origin's UTM zone: {outside_utm_count}")
print(f"Total /odom/pose/pose/position messages: {total_odom_messages}")

# Plotting the filtered x and y coordinates
plt.figure(figsize=(10, 5))
plt.plot(filtered_x_data, filtered_y_data, marker='o', linestyle='-', color='b', markersize=3)
plt.title('Filtered X and Y Positions from /odom')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)
plt.axis('equal')
plt.show()
#file_name='/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveA.csv'
file_name='/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_driveB.csv'
with open(file_name, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['X', 'Y'])  # Write header
    for x, y in zip(filtered_x_data, filtered_y_data):
        writer.writerow([x, y])