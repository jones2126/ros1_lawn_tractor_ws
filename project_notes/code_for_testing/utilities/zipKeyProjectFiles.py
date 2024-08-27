#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/utilities/zipKeyProjectFiles.py
"""
This script creates a zip file containing specified lawn tractor project files,
renaming them in the process. It searches for the files in their original locations
and places the resulting zip file in the user's Downloads folder. The zip file
is named with the current date for easy identification.
"""

import os
import zipfile
from datetime import date

def create_zip_file(file_mappings):
    print("Starting the zip file creation process...")

    # Get current date for the zip file name
    today = date.today().strftime("%Y%m%d")
    
    # Set the output directory to the Downloads folder
    output_dir = os.path.expanduser("~/Downloads")
    zip_filename = os.path.join(output_dir, f'lawn_tractor_files_{today}.zip')

    with zipfile.ZipFile(zip_filename, 'w') as zipf:
        for original_path, new_name in file_mappings:
            print(f"Processing file: {new_name}")
            if os.path.exists(original_path):
                zipf.write(original_path, new_name)
                print(f"  Added to zip: {new_name}")
            else:
                print(f"  Warning: File not found - {original_path}")

    print(f"Zip file created: {zip_filename}")

# File mappings: (original_path, new_name)
file_mappings = [
    ('/home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/gui/gui_main_v1.py', 'gui_main.py'),
    ('/home/tractor/github_projects/lawn_tractor_embedded_code/lawn_tractor_embedded_code/leftSpeed2024_v2/src/main.cpp', 'left_speed.cpp'),
    ('/home/tractor/github_projects/lawn_tractor_embedded_code/lawn_tractor_embedded_code/rightSpeed2024_v2/src/main.cpp', 'right_speed.cpp'),
    ('/home/tractor/github_projects/lawn_tractor_embedded_code/lawn_tractor_embedded_code/BNO085_ROS_IMU_Publisher/src/main.cpp', 'BNO085_ROS_IMU_Publisher.cpp'),
    ('/home/tractor/github_projects/lawn_tractor_embedded_code/lawn_tractor_embedded_code/ttgo_tractor_v5/src/main.cpp', 'ttgo_tractor.cpp'),
    ('/home/tractor/github_projects/lawn_tractor_embedded_code/lawn_tractor_embedded_code/ttgo_radio_control_v3/src/main.cpp', 'ttgo_radio.cpp'),
    ('/home/tractor/github_projects/lawn_tractor_embedded_code/lawn_tractor_embedded_code/05-serial.rules', 'udev_rules'),
    ('/home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/odom_from_wheel_and_gps.py', 'odom_from_wheel_and_gps.py'),
    ('/home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/ROS2portXfer.py', 'ROS2portXfer.py'),
    ('/home/tractor/ros1_lawn_tractor_ws/src/pure_pursuit/src/pure_pursuit.cpp', 'pure_pursuit.cpp'),
    ('/home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/launch/cub_cadet_location_2.launch', 'cub_cadet_location_2.launch'),
]

if __name__ == "__main__":
    create_zip_file(file_mappings)