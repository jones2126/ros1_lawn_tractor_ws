#!/usr/bin/env python
'''
script that reads the lat lon data from the bagfile mentioned about 
between a starting and endinging point in seconds, converts those positions 
to x and y coordinates and then plots them using pyplot.

$python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_review_using_rosbag_and_pyplot2.py

'''
import sys
sys.path.append('/home/tractor/catkin_ws/src/geonav_transform/src/')
#import geonav_transformations.geonav_conversions as gc  # from ChatGPT
#import geonav_transform.geonav_conversions as gc  # from original odom
#from sensor_msgs.msg import NavSatFix

import utm
import rosbag
import matplotlib.pyplot as plt
#import geonav_transform as gt
import geonav_transform.geonav_conversions as gt
import rospy

# Set the bag file path
bag_file_path = '/home/tractor/Downloads/2023-08-04-10-31-33_simple_square.bag'
bag = rosbag.Bag(bag_file_path)

# Set the time window in seconds
start_time = 1  # was 381 
end_time = 500

# Initialize lists to store X, Y coordinates and counters
x_coords = []
y_coords = []
colors = []
counter_running = 0
counter_moving = 0
counter_stopped = 0

# Define the origin latitude and longitude for transformation
origin_lat = 40.3452899
origin_lon = -80.1289039
# Convert to UTM coordinates
utm_coordinates = utm.from_latlon(origin_lat, origin_lon)

# Initialize the first timestamp
first_timestamp = None

# Iterate through the messages
for topic, msg, t in bag.read_messages(topics=['/fix']):
    if first_timestamp is None:
        first_timestamp = t.to_sec()

    timestamp_seconds = t.to_sec() - first_timestamp

    # Check if the message is within the time window
    if start_time <= timestamp_seconds <= end_time:
        lat = msg.latitude
        lon = msg.longitude

        # Set the origin if not already defined
        if origin_lat is None or origin_lon is None:
            origin_lat = lat
            origin_lon = lon

        # Convert lat/lon to X/Y using geonav transformations
        x, y = gt.ll2xy(lat, lon, origin_lat, origin_lon)
        x_coords.append(x)
        y_coords.append(y)

        # Determine the color and increment counters
        if 381 <= timestamp_seconds <= 438:
            colors.append('yellow')
            counter_running += 1
        elif 439 <= timestamp_seconds <= 461:
            colors.append('green')
            counter_moving += 1
        else:
            colors.append('red')
            counter_stopped += 1

# Plot the data
plt.scatter(x_coords, y_coords, c=colors)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Trajectory Plot')
plt.show()

# Print the counters
print("Running points:", counter_running)
print("Moving points:", counter_moving)
print("Stopped points:", counter_stopped)
print("utm_coordinates", utm_coordinates)
