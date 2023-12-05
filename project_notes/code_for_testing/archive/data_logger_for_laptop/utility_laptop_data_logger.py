#!/usr/bin/env python3  
#
'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/utility_laptop_data_logger.py

This program uses '' which requires 'sudo visudo' to have the following rule added:
tractor ALL=(ALL) NOPASSWD: /usr/sbin/iw dev wlo1 link


'''

import psutil
import time
from datetime import datetime
import os
import subprocess

# Function to get available disk storage space
def get_disk_space():
    disk_usage = psutil.disk_usage('/')
    return disk_usage.free

# Function to get CPU usage
def get_cpu_usage():
    return psutil.cpu_percent(interval=1)

# Function to get memory usage
def get_memory_usage():
    memory = psutil.virtual_memory()
    return memory.total, memory.used, memory.available

# Function to get Wi-Fi SSID and signal strength
def get_wifi_info():
    ssid, strength = "N/A", "N/A"
    if os.name == 'posix':  # Assuming Linux
        try:
            # Using 'sudo' to run the command might require the user to enter their password
            # This might not be suitable for a script running in the background
            networks = subprocess.check_output(['sudo', 'iw', 'dev', 'wlo1', 'link'], encoding='utf-8')
            
            for line in networks.split('\n'):
                if 'SSID' in line:
                    ssid = line.split('SSID:')[1].strip()
                if 'signal' in line:
                    strength = line.split('signal:')[1].strip().split(' ')[0]
                    
        except Exception as e:
            print(f"Error getting Wi-Fi info: {e}")
            
    return ssid, strength

# Function to get CPU temperature
def get_cpu_temp():
    try:
        # Run 'sensors' command and decode the output from bytes to string
        sensors_output = subprocess.check_output('sensors', encoding='utf-8')
        
        # Find the line with CPU temperature information
        for line in sensors_output.split('\n'):
            if 'k10temp-pci-00c3' in line:
                temp_line = next((l for l in sensors_output.split('\n') if 'temp1:' in l), None)
                if temp_line:
                    # Extract the temperature value and return it
                    temp_value = temp_line.split('+')[1].split('°C')[0].strip()
                    return temp_value                    
                    
    except Exception as e:
        print(f"Error getting CPU temperature: {e}")
        
    return "unavailable"


# Function to write CPU usage and other stats to a file
def log_stats(filename="cpu_usage_stats.csv"):
    #header = "Time,Avg_CPU_Util(%),Total_Memory(B),Used_Memory(B),Available_Memory(B),Wi-Fi_SSID,Wi-Fi_Signal_Strength(dBm),CPU_Temperature(°C)"
    header = "Time,Avg_CPU_Util(%),Total_Memory(B),Used_Memory(B),Available_Memory(B),Wi-Fi_SSID,Wi-Fi_Signal_Strength(dBm),CPU_Temperature(°C),Available_Disk_Space(B)"

    
    with open(filename, "w") as f:
        f.write(header + "\n")
    
    while True:
        cpu_usage = get_cpu_usage()
        total_memory, used_memory, available_memory = get_memory_usage()
        wifi_ssid, wifi_strength = get_wifi_info()
        cpu_temp = get_cpu_temp()
        avail_disk = get_disk_space()
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        #record = f"{timestamp},{cpu_usage},{total_memory},{used_memory},{available_memory},{wifi_ssid},{wifi_strength},{cpu_temp}"
        record = f"{timestamp},{cpu_usage},{total_memory},{used_memory},{available_memory},{wifi_ssid},{wifi_strength},{cpu_temp},{avail_disk}"

        
        with open(filename, "a") as f:
            f.write(record + "\n")
        
        time.sleep(60)  # Wait for 1 minute before taking the next reading

if __name__ == "__main__":
    filename = "cpu_usage_stats_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    log_stats(filename)
