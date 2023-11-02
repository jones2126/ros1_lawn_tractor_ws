#!/usr/bin/env python3
# python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_gps_to_cartesian.py
#

import json
import math
import matplotlib.pyplot as plt

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

def polar_to_cartesian(r, theta):
    x = round(r * math.cos(math.radians(90-theta)), 2)  # 90-theta is used because without it the orientation is East/West instead of North/Soutn
    y = round(r * math.sin(math.radians(90-theta)), 2)
    return x, y

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

def reorganize_list(cartesian_polygon, split_point):
    '''
    The list of points is simply a path of points that form a polygon.  I need to pick which point will be the first
    point for a driven path and it may not neccessarily be the first point in the list.  This function allows me
    to set the first point and then reorganize the list.
    '''
    if split_point < 0 or split_point >= len(cartesian_polygon):
        return "Invalid split point"
    
    return cartesian_polygon[split_point + 1:] + cartesian_polygon[:split_point + 1]



# Origin point - 62 Collins Dr
origin_lat = 40.34534080
origin_lon = -80.12894600

# Path to the JSON file
#file_path = '/home/aej/Downloads/polygon.json'
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/collins_dr_62.json'

# Read GPS coordinates from the JSON file
with open(file_path, 'r') as file:
    polygon_data = json.load(file)

# Converting GPS coordinates of the polygon to Cartesian coordinates in meters
cartesian_polygon = []
for point in polygon_data['missionPolygon']:
    distance = haversine(origin_lat, origin_lon, point['lat'], point['lng'])
    bearing = calculate_bearing(origin_lat, origin_lon, point['lat'], point['lng'])
    x, y = polar_to_cartesian(distance, bearing)
    cartesian_polygon.append((x, y))

print(cartesian_polygon)
# sort the list based on the starting point
reorganize_list(cartesian_polygon, 14)
print(cartesian_polygon)

# Call the plot_graph function to visualize the points
plot_graph(cartesian_polygon)

