#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/sendSpeedDataToSerial_v3.py
# 
import csv
import serial
import time
import sys

def open_serial_port(port='/dev/ttyACM0', baud_rate=460800, timeout=0):
    for attempt in range(5):  # Try 5 times
        try:
            ser = serial.Serial(port, baud_rate, timeout=timeout)
            print(f"Serial port opened successfully: {ser.name}")
            return ser
        except serial.SerialException as e:
            print(f"Attempt {attempt + 1}: Error opening serial port: {e}")
            time.sleep(2)  # Wait before retrying
    raise Exception("Failed to open serial port after multiple attempts")

print("Script started.")

try:
    print("Attempting to open serial port...")
    ser = open_serial_port()

    print("Opening CSV file...")
    with open('/home/tractor/wheel_data_right.csv', 'r') as file:
        print("CSV file opened successfully.")
        csv_reader = csv.reader(file)
        print("Skipping header row...")
        next(csv_reader)
        
        print("Converting CSV to list...")
        rows = list(csv_reader)
        total_records = len(rows)
        print(f"Total records: {total_records}")
        
        start_time = time.time()
        records_sent = 0
        target_rate = 12  # 10 = 10Hz; 50 = 50Hz
        sleep_time = 1 / target_rate
        total_sleep_time = 0
        
        print("Starting to process records...")
        for index, row in enumerate(rows):
            loop_start = time.time()
            
            # Extract and send only the current position (assuming it's the second column)
            current_position = row[1]
            data_string = f"{current_position}\n"
            ser.write(data_string.encode())
            records_sent += 1
            
            # Print progress every 20 seconds
            if time.time() - start_time >= 20:
                elapsed_time = time.time() - start_time
                rate = records_sent / elapsed_time
                remaining_records = total_records - (index + 1)
                print(f"Python: Records sent: {records_sent}, Remaining: {remaining_records}, Rate: {rate:.2f} Hz")
                start_time = time.time()
                records_sent = 0
            
            # Sleep to maintain target rate
            time_elapsed = time.time() - loop_start
            if time_elapsed < sleep_time:
                sleep_duration = sleep_time - time_elapsed
                total_sleep_time += sleep_duration
                time.sleep(sleep_duration)

except FileNotFoundError:
    print("CSV file not found. Please check the file path.")
    sys.exit(1)
except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
finally:
    if 'ser' in locals() and ser.is_open:
        print("Closing serial connection...")
        ser.close()
        print("Serial connection closed.")

print("All records processed.")
print(f"Total time spent sleeping to maintain rate: {total_sleep_time:.2f} seconds")
print(f"Average sleep time per record: {(total_sleep_time / total_records) * 1000:.2f} ms")