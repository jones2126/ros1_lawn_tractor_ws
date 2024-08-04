#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/speed_filter_and_plot.py
# file_path = '/home/tractor/wheel_data_right.csv'

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pykalman import KalmanFilter

# Load the data from the CSV file
file_path = '/home/tractor/wheel_data_right.csv'
df = pd.read_csv(file_path)

# Convert the timestamp from Unix epoch format to datetime
df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')

# Calculate seconds since the start of the data
df['seconds_since_start'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()

# Initialize the Kalman Filter
kf = KalmanFilter(initial_state_mean=0, n_dim_obs=1)
measurements = df['speed'].values
state_means, _ = kf.smooth(measurements)

# Define physical limits for speed
speed_min = -2.0
speed_max = 2.0

# Apply the Kalman filter to smooth the data
df['smoothed_speed'] = state_means

# Post-process to filter out physically implausible values
df['filtered_speed'] = df['smoothed_speed'].apply(lambda x: x if speed_min <= x <= speed_max else np.nan)

# Fill NaN values by forward and backward filling
df['filtered_speed'].fillna(method='ffill', inplace=True)
df['filtered_speed'].fillna(method='bfill', inplace=True)

# Plot the actual speed, the smoothed speed, and the filtered speed
plt.figure(figsize=(15, 8))
plt.plot(df['seconds_since_start'], df['speed'], label='Actual Speed', color='blue', alpha=0.5)
plt.plot(df['seconds_since_start'], df['smoothed_speed'], label='Smoothed Speed (Kalman Filter)', color='green', linestyle='dashed')
plt.plot(df['seconds_since_start'], df['filtered_speed'], label='Filtered Speed', color='red')

plt.xlabel('Seconds Since Start')
plt.ylabel('Speed')
plt.title('Speed Data Analysis with Kalman Filter and Physical Constraints')
plt.legend()
plt.grid(True)
plt.show()

# Save the filtered data to a new CSV file
output_file_path = '/home/tractor/wheel_data_right_filtered.csv'
df.to_csv(output_file_path, index=False)
print(f"Filtered data saved to {output_file_path}")
