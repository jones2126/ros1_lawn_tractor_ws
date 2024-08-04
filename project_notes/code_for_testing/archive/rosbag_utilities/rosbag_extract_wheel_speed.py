#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_extract_wheel_speed.py
import rosbag
import csv
import numpy as np
from collections import deque

# Path to your bagfile
bag_path = '/home/tractor/bagfiles/2024-07-23-17-07-34.bag'

# Output CSV files
left_wheel_csv = 'wheel_data_left.csv'
right_wheel_csv = 'wheel_data_right.csv'

# Function to filter outliers
def filter_outliers(data, window_size=5, threshold=5):
    filtered_data = []
    window = deque(maxlen=window_size)
    
    for value in data:
        if len(window) < window_size:
            window.append(value)
            filtered_data.append(value)
        else:
            window_avg = np.mean(window)
            if abs(value) <= threshold * abs(window_avg):
                filtered_data.append(value)
                window.append(value)
            else:
                filtered_data.append(window_avg)
                window.append(window_avg)
    
    return filtered_data

# Read the bag file and extract data
with rosbag.Bag(bag_path, 'r') as bag:
    left_wheel_data = []
    right_wheel_data = []
    
    for topic, msg, t in bag.read_messages(topics=['/wheel_data_left', '/wheel_data_right']):
        timestamp = t.to_sec()
        if topic == '/wheel_data_left':
            left_wheel_data.append([timestamp] + list(msg.data))
        elif topic == '/wheel_data_right':
            right_wheel_data.append([timestamp] + list(msg.data))

# Write data to CSV files
with open(left_wheel_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'current_position', 'total_rotations', 'delta', 'speed'])
    writer.writerows(left_wheel_data)

with open(right_wheel_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'current_position', 'total_rotations', 'delta', 'speed'])
    writer.writerows(right_wheel_data)

# Filter outliers in delta values
left_delta_filtered = filter_outliers([row[3] for row in left_wheel_data])
right_delta_filtered = filter_outliers([row[3] for row in right_wheel_data])

# Write filtered data to new CSV files
with open('left_wheel_filtered.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'current_position', 'total_rotations', 'delta_filtered', 'speed'])
    for original, filtered in zip(left_wheel_data, left_delta_filtered):
        writer.writerow([original[0], original[1], original[2], filtered, original[4]])

with open('right_wheel_filtered.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['timestamp', 'current_position', 'total_rotations', 'delta_filtered', 'speed'])
    for original, filtered in zip(right_wheel_data, right_delta_filtered):
        writer.writerow([original[0], original[1], original[2], filtered, original[4]])

print("Data extraction and filtering complete. Check the CSV files for results.")