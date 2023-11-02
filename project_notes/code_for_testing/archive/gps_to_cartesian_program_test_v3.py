#!/usr/bin/env python3  
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps_to_cartesian_program_test_v3.py
import math
import matplotlib.pyplot as plt
import json
import csv

def haversine(lat1, lon1, lat2, lon2):
    R = 6378137  # Radius of Earth in meters
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

def calculate_bearing(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    x = math.sin(d_lon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon))
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

def convert_polygon_to_cartesian(polygon, origin_lat, origin_lon):
    cartesian_points = []
    for point in polygon:
        lat, lon = point['lat'], point['lng']
        distance = haversine(origin_lat, origin_lon, lat, lon)
        bearing = calculate_bearing(origin_lat, origin_lon, lat, lon)
        #x, y = polar_to_cartesian(distance, bearing)
        x = distance * math.sin(math.radians(bearing))
        y = distance * math.cos(math.radians(bearing))
        cartesian_points.append((x, y))
    return cartesian_points    

def plot_graph(points):
    x_coords = [x for x, y in points]    # Extracting x and y coordinates from the points list
    y_coords = [y for x, y in points]
    plt.scatter(x_coords, y_coords, color='red')     # Plotting the points
    
    # Plotting line segments individually
    num_points = len(points)
    for i in range(num_points - 1):
        plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], 'b-')
    
    # Closing the polygon
    plt.plot([points[0][0], points[-1][0]], [points[0][1], points[-1][1]], 'b-')

    # Annotating the points
    for i, (x, y) in enumerate(points):
        plt.annotate(f'Point {i + 1}', (x, y), textcoords="offset points", xytext=(0,10), ha='center')
    
    plt.axis('equal')
    plt.show()    

# New Origin Point
new_origin_lat = 40.48524688166667
new_origin_lon = -80.332720941667
# Current Origin: origin_lat:=40.48524688166667, origin_lon:=-80.332720941667 - 62 Collins Dr

# Read GPS coordinates from the JSON file
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step1.json'
with open(file_path, 'r') as file:
    polygon_data = json.load(file)

mission_polygon = polygon_data['missionPolygon']
points_list2 = convert_polygon_to_cartesian(mission_polygon, new_origin_lat, new_origin_lon)
#plot_graph(points_list2)

# separate the list of points into the first section and second section
# Define the ending point
ending_point = 14  # for example, change this to your actual ending point index

# Split the points_list2 into points_list3 and points_list4
points_list3 = points_list2[:ending_point + 1]
points_list4 = points_list2[ending_point + 1:]
combined_list = points_list4 + points_list3
plot_graph(combined_list)
# Iterate over the points and write to the CSV file
csv_file_name = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62_A_step2.csv'
with open(csv_file_name, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["X", "Y"])
    for x, y in combined_list:
        writer.writerow([x, y])
print(f"Points written to {csv_file_name}")