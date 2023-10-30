#!/usr/bin/env python3  
#
'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/utility_cpu_usage_logger.py
or 
$ nohup python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/utility_cpu_usage_logger.py &
'''



import psutil
import time
from datetime import datetime

# Function to get CPU usage
def get_cpu_usage():
    return psutil.cpu_percent(interval=1)

# Function to write CPU usage and timestamp to a file
def log_cpu_usage(filename="cpu_usage_stats.txt"):
    while True:
        cpu_usage = get_cpu_usage()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(filename, "a") as f:
            f.write(f"Time, {timestamp}, Avg_CPU_Util, {cpu_usage}%\n")
        time.sleep(60)  # Wait for 1 minute before taking the next reading

if __name__ == "__main__":
    log_cpu_usage()
