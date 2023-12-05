#!/usr/bin/env python
'''
script that reads the lat lon data from the bagfile mentioned about 
between a starting and endinging point in seconds, converts those positions 
to x and y coordinates and then plots them using pyplot.

$python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_review_using_rosbag_and_pyplot2.py

'''
import rospy
import sys
sys.path.append('/home/tractor/catkin_ws/src/geonav_transform/src/')
#import geonav_transformations.geonav_conversions as gc  # from ChatGPT
import geonav_transform.geonav_conversions as gc  # from original odom

import rosbag
import matplotlib.pyplot as plt
import utm

# Path to the bag file
bag_file = '/home/tractor/Downloads/2023-08-04-10-31-33_simple_square.bag'
bag = rosbag.Bag(bag_file)

# Time range to extract
start_time = 381
end_time = 500

x_values = []
y_values = []
colors = []

# Iterate through the messages in the rosbag file
for topic, msg, t in bag.read_messages(topics=['/fix'], start_time=rospy.Time.from_sec(start_time), end_time=rospy.Time.from_sec(end_time)):
    timestamp = t.to_sec()
    print(f"Timestamp: {timestamp}") # Debugging line
    lat, lon = msg.latitude, msg.longitude
    print(f"Latitude: {lat}, Longitude: {lon}") # Debugging line

    # Convert lat/lon to UTM coordinates
    x, y, _, _ = utm.from_latlon(lat, lon)
    x_coords.append(x)
    y_coords.append(y)

    # Determine color based on time ranges
    color = None
    if 381 <= timestamp <= 438:
        color = 'yellow'
    elif 439 <= timestamp <= 461:
        color = 'green'
    elif 462 <= timestamp <= 500:
        color = 'red'

    if color:
        colors.append(color)
        print(f"Added point (x: {x}, y: {y}) with color: {color}") # Debugging line
    else:
        print(f"Skipped point (x: {x}, y: {y}) outside defined time ranges") # Debugging line


plt.scatter(x_values, y_values, c=colors)
plt.xlabel('X Coordinate (m)')
plt.ylabel('Y Coordinate (m)')
plt.title('Path Visualization')
plt.show()
