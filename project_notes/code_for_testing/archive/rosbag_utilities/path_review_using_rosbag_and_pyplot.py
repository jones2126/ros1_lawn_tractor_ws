#!/usr/bin/env python
'''
script that reads the lat lon data from the bagfile mentioned about 
between a starting and endinging point in seconds, converts those positions 
to x and y coordinates and then plots them using pyplot.

$python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_review_using_rosbag_and_pyplot.py

'''

import rospy
import rosbag
import os
# Import geonav tranformation module
import sys
sys.path.append('/home/tractor/catkin_ws/src/geonav_transform/src/')
#import geonav_transformations.geonav_conversions as gc  # from ChatGPT
import geonav_transform.geonav_conversions as gc  # from original odom
import matplotlib.pyplot as plt
import utm 

# Path to the rosbag file
bag_path = '/home/tractor/Downloads/2023-08-04-10-31-33_simple_square.bag'

# Time markers in seconds
start_time = 381
end_time = 500

# Time ranges for different statuses
time_running = (start_time, 438)
time_moving = (439, 461)
time_stopped = (462, end_time)

# Open the rosbag
bag = rosbag.Bag(bag_path)

# Lists to store x and y coordinates, colored by status
x_running, y_running = [], []
x_moving, y_moving = [], []
x_stopped, y_stopped = [], []

# Read messages
for topic, msg, t in bag.read_messages(topics=['/fix'], start_time=rosbag.Time(start_time), end_time=rosbag.Time(end_time)):
    time_seconds = t.to_sec()
    
    # Convert lat/lon to UTM
    x, y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
    
    # Assign coordinates to different statuses based on time
    if time_running[0] <= time_seconds <= time_running[1]:
        x_running.append(x)
        y_running.append(y)
    elif time_moving[0] <= time_seconds <= time_moving[1]:
        x_moving.append(x)
        y_moving.append(y)
    elif time_stopped[0] <= time_seconds <= time_stopped[1]:
        x_stopped.append(x)
        y_stopped.append(y)

# Plot the coordinates
plt.scatter(x_running, y_running, c='yellow', label='Running')
plt.scatter(x_moving, y_moving, c='green', label='Moving')
plt.scatter(x_stopped, y_stopped, c='red', label='Stopped')
plt.legend()
plt.xlabel('X Coordinate (m)')
plt.ylabel('Y Coordinate (m)')
plt.title('Position Plot')
plt.show()

# Close the bag
bag.close()
