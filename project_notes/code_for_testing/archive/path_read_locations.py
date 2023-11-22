#!/usr/bin/env python3
'''
The script is designed to print data from a dictionary based on the location name.
The dictionary is loaded by another function which reads the CSV file with location data.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_read_locations.py

'''

from path_load_location_data import load_location_data
# The path to the locations file
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/paths_locations.txt'
locations_data = load_location_data(file_path)
for location in locations_data:
    if location['location_name'] == 'Collins_Dr_62_Site_03':
        print(f"path and longitude: {location['csv_file_path']}, Longitude: {location['origin_lon']}")