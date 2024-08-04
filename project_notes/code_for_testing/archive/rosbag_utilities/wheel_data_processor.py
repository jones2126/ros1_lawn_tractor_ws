#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/wheel_data_processor.py
# 
import csv
from collections import deque
from time import time, sleep

QUEUE_SIZE = 4
REPORT_INTERVAL = 20  # 20 seconds
RESET_INTERVAL = 300  # 5 minutes

ROLLOVER_THRESHOLD = 15000
ROLLOVER_DROP = 8000

class WheelDataProcessor:
    def __init__(self):
        self.position_queue = deque(maxlen=QUEUE_SIZE)
        self.total_records = 0
        self.target_records = 0
        self.last_report_time = 0
        self.last_reset_time = 0

    def add_to_position_queue(self, position):
        self.position_queue.append(position)

    def filter_incoming_data(self, current_position):
        self.add_to_position_queue(current_position)
        
        if len(self.position_queue) == QUEUE_SIZE:
            condition1 = self.position_queue[2] > self.position_queue[1]
            condition2 = self.position_queue[1] > self.position_queue[0]
            condition3 = self.position_queue[3] < self.position_queue[2]
            condition4 = self.position_queue[1] > 13000
            condition5 = self.position_queue[2] > 15000
            condition6 = self.position_queue[3] > 2000

            if all([condition1, condition2, condition3, condition4, condition5, condition6]):
                self.target_records += 1
                print("Last 4 positions:", end=" ")
                for i, pos in enumerate(self.position_queue):
                    print(f"{i}:{pos:.0f}", end=" ")
                print()
                sleep(0.001)  # Short delay after sending a message

    def process_csv_data(self, csv_file_path):
        with open(csv_file_path, 'r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # Skip header row
            
            for row in csv_reader:
                current_time = time()
                current_position = float(row[1])  # Convert to float instead of int
                self.total_records += 1
                
                self.filter_incoming_data(current_position)
                
                # Report every REPORT_INTERVAL seconds
                if current_time - self.last_report_time >= REPORT_INTERVAL:
                    print(f"Total records processed: {self.total_records}, Target records: {self.target_records}")
                    sleep(0.001)
                    self.last_report_time = current_time
                
                # Reset counters every RESET_INTERVAL seconds
                if current_time - self.last_reset_time >= RESET_INTERVAL:
                    print("Resetting counters")
                    self.total_records = 0
                    self.target_records = 0
                    self.last_reset_time = current_time
                    sleep(0.001)

if __name__ == "__main__":
    processor = WheelDataProcessor()
    csv_file_path = '/home/tractor/wheel_data_right.csv'  # Update this path as needed
    processor.process_csv_data(csv_file_path)