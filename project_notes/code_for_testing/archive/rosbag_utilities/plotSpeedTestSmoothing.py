#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/plotSpeedTestSmoothing.py
'''
This script does the following:
    Reads the rosbag file and extracts speed data from both left and right wheel topics.
    Calculates the average speed.
    Applies a low-pass filter with different alpha values (0.1, 0.2, 0.3, 0.5).
    Plots the original speed and the filtered speeds for comparison.
    Plots the difference between the original and filtered speeds to show the effect of filtering.

After running this script, you'll see two plots:
    The first plot shows the original speed data along with the filtered data for different alpha values.
    The second plot shows the difference between the original and filtered speeds for each alpha value.

'''
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray

def low_pass_filter(data, alpha):
    filtered = np.zeros_like(data)
    filtered[0] = data[0]
    for i in range(1, len(data)):
        filtered[i] = alpha * data[i] + (1 - alpha) * filtered[i-1]
    return filtered

# Read the rosbag file
bag_file = '/home/tractor/bagfiles/2024-08-28-13-03-43.bag'
bag = rosbag.Bag(bag_file)

# Extract speed data
left_speeds = []
right_speeds = []
left_timestamps = []
right_timestamps = []

for topic, msg, t in bag.read_messages(topics=['/wheel_data_left', '/wheel_data_right']):
    if topic == '/wheel_data_left':
        left_speeds.append(msg.data[3])  # Assuming speed is the 4th element
        left_timestamps.append(t.to_sec())
    elif topic == '/wheel_data_right':
        right_speeds.append(msg.data[3])  # Assuming speed is the 4th element
        right_timestamps.append(t.to_sec())

bag.close()

print(f"Number of left speed values: {len(left_speeds)}")
print(f"Number of right speed values: {len(right_speeds)}")

# Convert to numpy arrays
left_speeds = np.array(left_speeds)
right_speeds = np.array(right_speeds)

# Use the shorter array length
min_length = min(len(left_speeds), len(right_speeds))
left_speeds = left_speeds[:min_length]
right_speeds = right_speeds[:min_length]

# Calculate average speed
avg_speeds = (left_speeds + right_speeds) / 2

# Use left timestamps (they should be very close to right timestamps)
timestamps = np.array(left_timestamps[:min_length])

# Apply low-pass filter with different alpha values
alphas = [0.1, 0.2, 0.3, 0.5]
filtered_speeds = {alpha: low_pass_filter(avg_speeds, alpha) for alpha in alphas}

# Plotting
plt.figure(figsize=(15, 10))
plt.plot(timestamps, avg_speeds, label='Original')
for alpha, speeds in filtered_speeds.items():
    plt.plot(timestamps, speeds, label=f'Alpha = {alpha}')

plt.xlabel('Time (s)')
plt.ylabel('Speed')
plt.title('Original vs Filtered Speeds')
plt.legend()
plt.grid(True)
plt.show()

# Plot the difference between original and filtered speeds
plt.figure(figsize=(15, 10))
for alpha, speeds in filtered_speeds.items():
    plt.plot(timestamps, avg_speeds - speeds, label=f'Alpha = {alpha}')

plt.xlabel('Time (s)')
plt.ylabel('Speed Difference')
plt.title('Difference between Original and Filtered Speeds')
plt.legend()
plt.grid(True)
plt.show()