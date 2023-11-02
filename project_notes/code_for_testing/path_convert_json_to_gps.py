#!/usr/bin/env python3
# python3 /home/aej/Downloads/utility_convert_json_to_gps.py
import json
import csv

# Path to the JSON file
#file_path = '/home/aej/Downloads/polygon.json'
file_path = '/home/aej/Downloads/collins_dr_62.json'

# Read GPS coordinates from the JSON file
with open(file_path, 'r') as file:
    polygon_data = json.load(file)

# Path to the output CSV file
#output_csv_path = '/home/aej/Downloads/polygon_output.csv'
output_csv_path = '/home/aej/Downloads/collins_dr_62.csv'

# Writing to the CSV file
with open(output_csv_path, 'w', newline='') as csvfile:
    fieldnames = ['latitude', 'longitude', 'name']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

    writer.writeheader()
    for i, point in enumerate(polygon_data['missionPolygon'], start=1):
        row = {'latitude': point['lat'], 'longitude': point['lng'], 'name': f'Point {i}'}
        writer.writerow(row)

print(f'Data has been written to {output_csv_path}')
