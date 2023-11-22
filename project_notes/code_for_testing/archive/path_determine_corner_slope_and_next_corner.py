#!/usr/bin/env python3
'''
The script is designed to:
- Read .csv file to get x, y points
- Craft a polygon from those points

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_determine_corner_slope_and_next_corner.py

'''
import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

from path_generator_utilities import (
    load_locations_db, find_southernmost_point_with_index, calculate_distance, haversine, calculate_bearing,  
    calculate_slope, write_to_csv, build_points_list, convert_gps_to_cartesian, plot_graph
)
import pandas as pd

# get the filename from the locations db for the 'site' you want to work with
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/paths_locations.txt'   # The path to the locations file/db
locations_data = load_locations_db(file_path) 
site = 'Collins_Dr_62_Site_01'
for location in locations_data:
    if location['location_name'] == site:
        csv_file_path = location['csv_file_path']
        origin_lat = location['origin_lat']
        origin_lon = location['origin_lon']
        print(f"path and longitude: {location['csv_file_path']}, Longitude: {location['origin_lon']}")
print(f"reading file: {csv_file_path}")
points, num_points = build_points_list(csv_file_path, 'lat', 'lng')

# Find the southernmost point and its index
southern_most_point, southern_most_point_index = find_southernmost_point_with_index(points)
print(f"southern_most_point: {southern_most_point}")

# Reorder the list starting from the southernmost point in a counter clockwise direction
# This assumes that the next point in the list after southernmost point is in the counter clockwise direction
points_reordered = points[southern_most_point_index:] + points[:southern_most_point_index]

# Create the 'distances_from_south' list
distances_from_south = []

for point in points_reordered:
    lat, lon = point
    # Calculate the distance from the southernmost point
    distance = calculate_distance(southern_most_point[0], southern_most_point[1], lat, lon)
    # Calculate the slope of the line between the current point and the southernmost point
    slope = calculate_slope(southern_most_point[0], southern_most_point[1], lat, lon)
    # Append the data to the list
    distances_from_south.append((lat, lon, distance, slope))

# send output to .csv
# Define the headers for the CSV file
headers = ['Latitude', 'Longitude', 'Distance_km', 'Slope']
output_csv_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_southwest_corner.csv'
write_to_csv(distances_from_south, headers, output_csv_path)

start_index = 380
end_index = 400
interesting_points = distances_from_south[start_index:end_index + 1]
print("interesting_points data type:", type(interesting_points))
#print(interesting_points)

# Convert the list to a DataFrame
interesting_points_df = pd.DataFrame(interesting_points, columns=['lat', 'lng', 'Distance_km', 'Slope'])

# calculate the x, y coordinates based on the lat, lon and add the data to the list
interesting_points_with_x_and_y = convert_gps_to_cartesian(interesting_points_df, origin_lat, origin_lon)
print("interesting_points_with_x_and_y data type:", type(interesting_points_with_x_and_y))
output_csv_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_interesting_pts.csv'
interesting_points_with_x_and_y.to_csv(output_csv_path, index=False)  # index=False to exclude the index from the CSV
print(f"file written to: {output_csv_path}")

x_coords = interesting_points_with_x_and_y['X'].tolist()
y_coords = interesting_points_with_x_and_y['Y'].tolist()

# Combine them into a list of (X, Y) tuples
points = list(zip(x_coords, y_coords))
#plot_graph(points, script_name, output_csv_path, keep_plot_open='Y', annotate_points='N') 
# after manual inspection I will make cornertwo(40.4852321,-80.3322439)

