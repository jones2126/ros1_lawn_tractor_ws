#!/usr/bin/env python
'''
Script that reads the lat, lon data from the /fix topic and x, y positions from 
/odom/pose/pose/position topic in a ROS bag file, between a starting and ending 
point in seconds, converts those positions to x, y coordinates, and then plots them 
using pyplot.
'''
import matplotlib.pyplot as plt
import rosbag
import rospy
import geonav_transform.geonav_conversions as gc
import utm

# Origin details
origin_lat = 40.485509842
origin_lon = -80.332308247

# Get UTM zone for the origin
origin_utm_zone = utm.from_latlon(origin_lat, origin_lon)[2]

# Path to the ROS bag file
bag_path = '/home/tractor/bagfiles/62Collins_perimiter_1_2024-04-17-18-34-40.bag'
bag = rosbag.Bag(bag_path)

# Time range for reading messages
start_time = 1
end_time = 9999

first_timestamp = None
outside_utm_count = 0
total_fix_messages = 0
total_odom_messages = 0

# Containers to store lat, lon, x, y data
lat_data = []
lon_data = []
x_data = []
y_data = []

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

            if current_utm_zone != origin_utm_zone:
                outside_utm_count += 1
            
            total_fix_messages += 1
        
        if topic == '/odom':
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            x_data.append(x)
            y_data.append(y)
            total_odom_messages += 1            

bag.close()

# Coordinates for the line segments
line_segments = [(1.0, -25.3), (-30, -37.2), (-45, -3.4), (-12.3, 14.7), (1.0, -25.3)]

# Plotting the x and y coordinates and the line segments
plt.figure(figsize=(10, 5))
plt.plot(x_data, y_data, marker='o', linestyle='-', color='b')
plt.plot(*zip(*line_segments), linestyle='-', color='r', linewidth=2)  # Draws the line segments in red
plt.title('X and Y Positions from /odom/pose/pose/position with Line Segments')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.grid(True)
plt.axis('equal')  # Ensures equal aspect ratio for the plot

plt.show()
