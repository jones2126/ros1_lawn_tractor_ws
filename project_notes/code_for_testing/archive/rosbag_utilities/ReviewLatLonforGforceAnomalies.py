#!/usr/bin/env python3

'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/ReviewLatLonforGforceAnomalies.py

 extract the lat, lon data from your ROSBAG file and calculate the acceleration between points.
 store the acceleration over time data and include the timestamps.  Then using that data, we need to filter out the time 
 periods where GPS RTK status != 2.  Then let's bucket the data that shows what the typical and unusual acceleration points are.

This script does the following:

Stores all data, including timestamps and RTK status, in a pandas DataFrame.
Calculates speeds and accelerations.
Filters the data to include only points where RTK status is 2.
Calculates statistical measures of the acceleration data.
Defines thresholds for unusual accelerations using the interquartile range method.
Categorizes accelerations as "Unusually Low", "Normal", or "Unusually High".
Plots a histogram of accelerations with the unusual thresholds marked.
Prints out details of the unusual acceleration points.
Saves the filtered data (RTK fix only) to a CSV file for further analysis if needed.

bag_file = '/home/tractor/bagfiles/2024-08-27-14-33-33.bag'
'''
import rosbag
from rosbag import Bag
import numpy as np
import matplotlib.pyplot as plt
from geopy import distance
from datetime import datetime
import pandas as pd
from scipy import stats
from datetime import datetime, timezone
import pytz

def convert_timestamp(timestamp):
    # Convert to datetime object
    dt = datetime.fromtimestamp(timestamp, tz=timezone.utc)
    
    # Convert to Eastern Time
    eastern = pytz.timezone('US/Eastern')
    dt_eastern = dt.astimezone(eastern)
    
    return dt_eastern

def calculate_acceleration(speeds, times):
    accelerations = np.diff(speeds) / np.diff(times)
    return np.append(accelerations, 0)

bag_file = '/home/tractor/bagfiles/2024-08-27-14-33-33.bag'

data = []

with Bag(bag_file, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        data.append({
            'timestamp': t.to_sec(),
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'status': msg.status.status  # Assuming RTK status is in this field
        })

df = pd.DataFrame(data)
df['time_diff'] = df['timestamp'].diff()
df['distance'] = df.apply(lambda row: distance.distance(
    (df.loc[row.name-1, 'latitude'], df.loc[row.name-1, 'longitude']),
    (row['latitude'], row['longitude'])
).meters if row.name > 0 else 0, axis=1)

df['speed'] = df['distance'] / df['time_diff']
df['acceleration'] = calculate_acceleration(df['speed'].values, df['time_diff'].values)

# Filter data where RTK status is 2
df_rtk = df[df['status'] == 2].copy()

print(f"Total data points: {len(df)}")
print(f"Data points with RTK fix: {len(df_rtk)}")

# Analyze acceleration data
acceleration_data = df_rtk['acceleration'].dropna()

# Calculate statistics
mean_acc = acceleration_data.mean()
std_acc = acceleration_data.std()
median_acc = acceleration_data.median()
q1_acc = acceleration_data.quantile(0.25)
q3_acc = acceleration_data.quantile(0.75)
iqr_acc = q3_acc - q1_acc

# Define thresholds for unusual accelerations
lower_threshold = q1_acc - 1.5 * iqr_acc
upper_threshold = q3_acc + 1.5 * iqr_acc

print("\nAcceleration Statistics:")
print(f"Mean: {mean_acc:.4f} m/s²")
print(f"Standard Deviation: {std_acc:.4f} m/s²")
print(f"Median: {median_acc:.4f} m/s²")
print(f"Q1: {q1_acc:.4f} m/s²")
print(f"Q3: {q3_acc:.4f} m/s²")
print(f"IQR: {iqr_acc:.4f} m/s²")

print(f"\nUnusual Acceleration Thresholds:")
print(f"Lower: {lower_threshold:.4f} m/s²")
print(f"Upper: {upper_threshold:.4f} m/s²")

# Classify accelerations
df_rtk['acceleration_category'] = pd.cut(df_rtk['acceleration'], 
                                         bins=[-np.inf, lower_threshold, upper_threshold, np.inf],
                                         labels=['Unusually Low', 'Normal', 'Unusually High'])

# Count occurrences in each category
category_counts = df_rtk['acceleration_category'].value_counts()
print("\nAcceleration Categories:")
print(category_counts)

# Plot histogram of accelerations
plt.figure(figsize=(12, 6))
plt.hist(acceleration_data, bins=50, edgecolor='black')
plt.title('Histogram of Accelerations (RTK Fix Only)')
plt.xlabel('Acceleration (m/s²)')
plt.ylabel('Frequency')
plt.axvline(lower_threshold, color='r', linestyle='dashed', linewidth=2, label='Unusual Thresholds')
plt.axvline(upper_threshold, color='r', linestyle='dashed', linewidth=2)
plt.legend()
plt.show()

# Print unusual acceleration points
unusual_acc = df_rtk[(df_rtk['acceleration'] < lower_threshold) | (df_rtk['acceleration'] > upper_threshold)]
print("\nUnusual Acceleration Points:")
for _, row in unusual_acc.iterrows():
    print(f"Timestamp: {convert_timestamp(row['timestamp'])}, "
          f"Acceleration: {row['acceleration']:.4f} m/s², "
          f"Lat: {row['latitude']}, Lon: {row['longitude']}")

# Optional: Save filtered data to CSV
df_rtk.to_csv('rtk_acceleration_data.csv', index=False)