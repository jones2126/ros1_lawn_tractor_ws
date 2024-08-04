#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/sendSpeedDataToSerial.py
# 
import csv
import serial
import time
import sys

def open_serial_port(port='/dev/ttyACM0', baud_rate=460800, timeout=1):
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
        target_rate = 100  # 50 Hz
        sleep_time = 1 / target_rate

        
        print("Starting to process records...")
        for index, row in enumerate(rows):
            try:
                loop_start = time.time()
                
                data_string = ','.join(row) + '\n'
                ser.write(data_string.encode())
                records_sent += 1
                
                # Check for Arduino response
                if ser.in_waiting:
                    response = ser.readline().decode().strip()
                    if response:
                        print(f"Arduino: {response}")
                
                # Print progress every 20 seconds
                if time.time() - start_time >= 20:
                    elapsed_time = time.time() - start_time
                    rate = records_sent / elapsed_time
                    remaining_records = total_records - (index + 1)
                    print(f"Python: Records sent: {records_sent}, Remaining: {remaining_records}, Rate: {rate:.2f} Hz")
                    start_time = time.time()
                    records_sent = 0
                
                # Sleep to maintain rate
                time_elapsed = time.time() - loop_start
                if time_elapsed < sleep_time:
                    time.sleep(sleep_time - time_elapsed)
                
            except serial.SerialException as e:
                print(f"Serial error occurred: {e}")
                ser = open_serial_port()  # Try to reopen the port
                
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