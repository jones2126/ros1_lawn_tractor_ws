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
bag_path = "/home/tractor/Downloads/2023-08-04-10-31-33_simple_square.bag"

# Time intervals
start_time = 381
end_time = 500

# Lists to store coordinates
x_running, y_running = [], []
x_moving, y_moving = [], []
x_stopped, y_stopped = [], []

# Open the bag file
with rosbag.Bag(bag_path, "r") as bag:
    # Iterate through the messages in the /fix topic within the specified time range
    for topic, msg, t in bag.read_messages(topics=['/fix'], start_time=rospy.Time.from_sec(start_time), end_time=rospy.Time.from_sec(end_time)):
        # Convert the timestamp to seconds
        timestamp_seconds = t.to_sec()

        # Convert latitude and longitude to UTM coordinates
        x, y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        # Categorize the coordinates based on the time intervals
        if 381 <= timestamp_seconds <= 438:
            x_running.append(x)
            y_running.append(y)
        elif 439 <= timestamp_seconds <= 461:
            x_moving.append(x)
            y_moving.append(y)
        elif 462 <= timestamp_seconds <= 500:
            x_stopped.append(x)
            y_stopped.append(y)

# Plot the coordinates with different colors for each category
plt.scatter(x_running, y_running, c='yellow', label='Running')
plt.scatter(x_moving, y_moving, c='green', label='Moving')
plt.scatter(x_stopped, y_stopped, c='red', label='Stopped')

plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.title('Trajectory Visualization')
plt.show()
