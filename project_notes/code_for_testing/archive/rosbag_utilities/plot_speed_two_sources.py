#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/plot_speed_two_sources.py
# 
import pandas as pd
import matplotlib.pyplot as plt

# Define the file paths
original_file_path = '/home/tractor/wheel_data_right.csv'
corrected_file_path = '/home/tractor/wheel_data_right_corrected.csv'

# Load the data
df_original = pd.read_csv(original_file_path)
df_corrected = pd.read_csv(corrected_file_path)

# Calculate the time elapsed for both datasets
start_time_original = df_original['timestamp'].iloc[0]
df_original['time_elapsed'] = df_original['timestamp'] - start_time_original

start_time_corrected = df_corrected['timestamp'].iloc[0]
df_corrected['time_elapsed'] = df_corrected['timestamp'] - start_time_corrected

# Plot the speed from both datasets
plt.figure(figsize=(15, 8))

plt.plot(df_original['time_elapsed'], df_original['speed'], label='Original Speed', color='blue', alpha=0.5)
plt.plot(df_corrected['time_elapsed'], df_corrected['speed'], label='Corrected Speed', color='red', alpha=0.5)

plt.xlabel('Time Elapsed (seconds)')
plt.ylabel('Speed (m/s)')
plt.title('Speed Comparison: Original vs Corrected Data')
plt.legend()
plt.grid(True)
plt.show()
