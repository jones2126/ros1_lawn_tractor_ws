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
import subprocess
import shutil

# Function to get current from BAT0-acpi-0 curr1
def get_sensor_output():
    try:
        sensor_output = subprocess.check_output("sensors", encoding="utf-8")
        return sensor_output
    except Exception as e:
        print(f"Error getting sensor output: {e}")
        return ""

# Function to get voltage from BAT0-acpi-0 in0

def get_voltage_and_current(sensor_output):
    try:
        inside_bat0_section = False
        voltage = "unavailable"
        current = "unavailable"
        
        # Find the lines with voltage and current information
        for line in sensor_output.split('\n'):
            if 'BAT0-acpi-0' in line:
                inside_bat0_section = True
            elif inside_bat0_section:
                if 'in0:' in line:
                    voltage_line = line.split('in0:')[1]
                    voltage = voltage_line.split('V')[0].strip()
                elif 'curr1:' in line:
                    current_line = line.split('curr1:')[1]
                    current = current_line.split('A')[0].strip()
            elif inside_bat0_section and line.strip() == "":
                # Exit the BAT0-acpi-0 section if an empty line is encountered
                inside_bat0_section = False
                
        return voltage, current
        
    except Exception as e:
        print(f"Error getting voltage and current: {e}")
        return "unavailable", "unavailable"

def get_sensor_data():
    try:
        inside_bat0_section = False
        inside_k10temp_section = False
        inside_acpitz_section = False
        voltage = "unavailable"
        current = "unavailable"
        temp_k10temp = "unavailable"
        temp_acpitz_1 = "unavailable"
        temp_acpitz_5 = "unavailable"
        sensor_output = subprocess.check_output("sensors", encoding="utf-8")
        # Find the lines with voltage, current, and temperature information
        for line in sensor_output.split('\n'):
            if 'BAT0-acpi-0' in line:
                inside_bat0_section = True
                inside_k10temp_section = False
                inside_acpitz_section = False
            elif 'k10temp-pci-00c3' in line:
                inside_k10temp_section = True
                inside_bat0_section = False
                inside_acpitz_section = False
            elif 'acpitz-acpi-0' in line:
                inside_acpitz_section = True
                inside_bat0_section = False
                inside_k10temp_section = False
            elif inside_bat0_section:
                if 'in0:' in line:
                    voltage_line = line.split('in0:')[1]
                    voltage = voltage_line.split('V')[0].strip()
                elif 'curr1:' in line:
                    current_line = line.split('curr1:')[1]
                    current = current_line.split('A')[0].strip()
            elif inside_k10temp_section and 'temp1:' in line:
                temp_k10temp = line.split('+')[1].split('°C')[0].strip()
            elif inside_acpitz_section:
                if 'temp1:' in line:
                    temp_acpitz_1 = line.split('+')[1].split('°C')[0].strip()
                elif 'temp5:' in line:
                    temp_acpitz_5 = line.split('+')[1].split('°C')[0].strip()
                
        return voltage, current, temp_k10temp, temp_acpitz_1, temp_acpitz_5
        
    except Exception as e:
        print(f"Error getting sensor data: {e}")
        return "unavailable", "unavailable", "unavailable", "unavailable", "unavailable"

def get_disk_space():
    disk_usage = psutil.disk_usage('/')
    avail_disk = round(disk_usage.free / 1_000_000_000, 2)
    return avail_disk

def get_cpu_usage():
    return psutil.cpu_percent(interval=1)

def get_memory_info():
    memory = psutil.virtual_memory()
    total_memory = round(memory.total / 1_000_000_000, 2)
    used_memory = round(memory.used / 1_000_000_000, 2)
    available_memory = round(memory.available / 1_000_000_000, 2)
    return total_memory, used_memory, available_memory   

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

def log_stats(filename="cpu_usage_stats.csv"):
    header = (
        "Time,Avg_CPU_Util(%),Total_Memory(GB),Used_Memory(GB),Available_Memory(GB),"
        "Wi-Fi_SSID,Wi-Fi_Signal_Strength(dBm),CPU_Temperature(°C),Available_Disk_Space(GB),"
        "CPU_Temp1(°C),Zone1_Temp(°C),Zone5_Temp(°C),Voltage(V),Current(A)"
    )
    with open(filename, "w") as f:
        f.write(header + "\n")
    
    while True:
        voltage, current, cpu_temp, zone1_temp, zone5_temp = get_sensor_data()
        cpu_usage = get_cpu_usage()
        total_memory, used_memory, available_memory = get_memory_info()
        wifi_ssid, wifi_strength = get_wifi_info()
        avail_disk = get_disk_space()
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
