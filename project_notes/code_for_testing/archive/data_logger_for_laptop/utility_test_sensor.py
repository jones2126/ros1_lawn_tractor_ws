#!/usr/bin/env python3  
#
'''
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/utility_test_sensor.py

This program uses 'sudo /usr/sbin/iw dev wlo1 link' which requires 'sudo visudo' to have the following rule added:
tractor ALL=(ALL) NOPASSWD: /usr/sbin/iw dev wlo1 link


'''

import psutil
import time
from datetime import datetime
import os
import subprocess
import shutil

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


def get_memory_info():
    memory = psutil.virtual_memory()
    total_memory = round(memory.total / 1_000_000_000, 2)
    used_memory = round(memory.used / 1_000_000_000, 2)
    available_memory = round(memory.available / 1_000_000_000, 2)
    return total_memory, used_memory, available_memory    

if __name__ == "__main__":
    voltage, current, temp_cpu, temp_zone1, temp_zone5 = get_sensor_data()
    print("Voltage:", voltage)
    print("Current:", current)
    print("Temp temp_cpu:", temp_cpu)
    print("Temp temp_zone1:", temp_zone1)
    print("Temp temp_zone5:", temp_zone5)

    total_memory, used_memory, available_memory = get_memory_info()
    print("total_memory:", total_memory)
    print("used_memory:", used_memory)
    print("available_memory:", available_memory)    

