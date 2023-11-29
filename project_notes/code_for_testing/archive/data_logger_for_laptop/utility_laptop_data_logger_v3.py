#!/usr/bin/env python3  
#
'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/utility_laptop_data_logger.py

This program uses 'sudo /usr/sbin/iw dev wlo1 link' which requires 'sudo visudo' to have the following rule added:
tractor ALL=(ALL) NOPASSWD: /usr/sbin/iw dev wlo1 link


'''

import psutil
import time
from datetime import datetime
import os
import
# Function to get current from BAT0-acpi-0 curr1
def get_current():
    try:
        # Run 'sensors' command and decode the output from bytes to string
        sensors_output = subprocess.check_output('sensors', encoding='utf-8')
        
        # Find the line with current information
        for line in sensors_output.split('\n'):
            if 'BAT0-acpi-0' in line and 'curr1:' in line:
                current = line.split('A')[0].split(' ')[-1].strip()
                return current
                
        return "unavailable"
        
    except Exception as e:
        print(f"Error getting current: {e}")
        return "unavailable"

# Function to get voltage from BAT0-acpi-0 in0
def get_voltage():
    try:
        # Run 'sensors' command and decode the output from bytes to string
        sensors_output = subprocess.check_output('sensors', encoding='utf-8')
        
        # Find the line with voltage information
        for line in sensors_output.split('\n'):
            if 'BAT0-acpi-0' in line and 'in0:' in line:
                voltage = line.split('V')[0].split(' ')[-1].strip()
                return voltage
                
        return "unavailable"
        
    except Exception as e:
        print(f"Error getting voltage: {e}")
        return "unavailable"
 subprocess

# Function to get available disk storage space
def get_disk_space():
    disk_usage = psutil.disk_usage('/')
    return disk_usage.free

def get_disk_space():
    total_disk, used_disk, avail_disk = shutil.disk_usage("/")
    return avail_disk / 1_000_000_000 

# Function to get CPU usage
def get_cpu_usage():
    return psutil.cpu_percent(interval=1)

# Function to get memory usage
def get_memory_info():
    total_memory, used_memory, available_memory = shutil.disk_usage("/")
    return total_memory / 1_000_000_000, used_memory / 1_000_000_000, available_memory / 1_000_000_000    

# Function to get Wi-Fi SSID and signal strength
def get_wifi_info():
    ssid, strength = "N/A", "N/A"
    if os.name == 'posix':  # Assuming Linux
        try:
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
        
        # Initialize temperature values
        temp_k10temp = "unavailable"
        temp_acpitz_1 = "unavailable"
        temp_acpitz_5 = "unavailable"
        
        # Find the lines with CPU temperature information
        for line in sensors_output.split('\n'):
            if 'k10temp-pci-00c3' in line and 'temp1:' in line:
                temp_k10temp = line.split('+')[1].split('°C')[0].strip()
            elif 'acpitz-acpi-0' in line and 'temp1:' in line:
                temp_acpitz_1 = line.split('+')[1].split('°C')[0].strip()
            elif 'acpitz-acpi-0' in line and 'temp5:' in line:
                temp_acpitz_5 = line.split('+')[1].split('°C')[0].strip()
                
        return temp_k10temp, temp_acpitz_1, temp_acpitz_5
        
    except Exception as e:
        print(f"Error getting CPU temperature: {e}")
        return "unavailable", "unavailable", "unavailable"
         temp_value = temp_line.split('+')[1].split('°C')[0].strip()
                    return temp_value                    
                    
    except Exception as e:
        print(f"Error getting CPU temperature: {e}")
        
    return "unavailable"


# Function to write stats to a file
def log_stats(filename="cpu_usage_stats.csv"):
    header = (
        "Time,Avg_CPU_Util(%),Total_Memory(GB),Used_Memory(GB),Available_Memory(GB),"
        "Wi-Fi_SSID,Wi-Fi_Signal_Strength(dBm),CPU_Temperature(°C),Available_Disk_Space(GB),"
        "CPU_Temp1(°C),Zone1_Temp(°C),Zone5_Temp(°C),Voltage(V),Current(A)"
    )
    with open(filename, "w") as f:
        f.write(header + "\n")
    
    while True:
        cpu_usage = get_cpu_usage()
        total_memory, used_memory, available_memory = get_memory_usage()
        wifi_ssid, wifi_strength = get_wifi_info()
        cpu_temp, zone1_temp, zone5_temp = get_cpu_temp()
        avail_disk = get_disk_space()
        voltage = get_voltage()
        current = get_current()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        record = (
            f"{timestamp},{cpu_usage},{total_memory},{used_memory},{available_memory},{wifi_ssid},{wifi_strength},"
            f"{cpu_temp},{avail_disk},{cpu_temp},{zone1_temp},{zone5_temp},{voltage},{current}"
        )
        with open(filename, "a") as f:
            f.write(record + "\n")            
        time.sleep(60)  # Wait for 1 minute before taking the next reading

if __name__ == "__main__":
    filename = "cpu_usage_stats_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
    log_stats(filename)
