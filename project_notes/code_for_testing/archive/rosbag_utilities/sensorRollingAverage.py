#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/sensorRollingAverage.py
# 
import csv
import time
from collections import deque

def calculate_rolling_average(data):
    return sum(data) / len(data) if data else 0

original_file_path = '/home/tractor/wheel_data_right.csv'
window_size = 5  # Size of the rolling window for delta average
delta_threshold = 1000  # Threshold for printing values
output_rate = 1 / 50  # 50 Hz

readings = []  # Stack to store readings
deltas = deque(maxlen=window_size)
last_print_time = 0

def get_reading(readings, index, default='N/A'):
    try:
        return f"{readings[index]:.2f}"
    except IndexError:
        return default

print(f"Starting to process file: {original_file_path}")

with open(original_file_path, 'r') as csvfile:
    csv_reader = csv.reader(csvfile)
    next(csv_reader)  # Skip header row if it exists
    
    records_processed = 0
    for index, row in enumerate(csv_reader, start=1):
        if 300 <= index <= 400:
            records_processed += 1
            current_reading = float(row[1])  # Assuming the sensor reading is in the second column
            readings.append(current_reading)
            
            if len(readings) > 5:
                readings.pop(0)  # Remove the oldest reading if we have more than 5
            
            if len(readings) >= 2:
                delta = readings[-1] - readings[-2]
                deltas.append(delta)
                rolling_avg = calculate_rolling_average(deltas)
                
                print(f"N: {get_reading(readings, -1)}, " +
                      f"N-1: {get_reading(readings, -2)}, " +
                      f"N-2: {get_reading(readings, -3)}, " +
                      f"N-3: {get_reading(readings, -4)}, " +
                      f"N-4: {get_reading(readings, -5)}, " +
                      f"Delta: {delta:.2f}, Avg Delta: {rolling_avg:.2f}")
                
                if abs(delta - rolling_avg) >= delta_threshold:
                    print("THRESHOLD EXCEEDED")
            
            time.sleep(0.02)  # Sleep for 20ms (50Hz)
        elif index > 400:
            break  # Stop processing after record 400

print(f"Finished processing. Records processed: {records_processed}")
print(f"Final readings: {readings}")
print(f"Final deltas: {list(deltas)}")