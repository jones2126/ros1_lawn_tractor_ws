#!/usr/bin/env python3  
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps_to_cartesian_program_test.py
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
    x = r * math.sin(math.radians(theta))
    y = r * math.cos(math.radians(theta))
    return x, y

def calculate_new_lat_lon(lat, lon, distance, bearing):
    R = 6378137  # Radius of Earth in meters
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    angular_distance = distance / R
    new_lat_rad = lat_rad + angular_distance * math.cos(math.radians(bearing))
    new_lon_rad = lon_rad + (angular_distance * math.sin(math.radians(bearing)) / math.cos(new_lat_rad))
    new_lat = math.degrees(new_lat_rad)
    new_lon = math.degrees(new_lon_rad)
    return new_lat, new_lon

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

# Calculate Points 2, 3, and 4
point_2_lat, point_2_lon = calculate_new_lat_lon(new_origin_lat, new_origin_lon, 30, 90)
point_3_lat, point_3_lon = calculate_new_lat_lon(point_2_lat, point_2_lon, 30, 0)
point_4_lat, point_4_lon = calculate_new_lat_lon(point_3_lat, point_3_lon, 30, 270)

# GPS Coordinates
gps_points = {
    "Point 1 (Origin)": (new_origin_lat, new_origin_lon),
    "Point 2": (point_2_lat, point_2_lon),
    "Point 3": (point_3_lat, point_3_lon),
    "Point 4": (point_4_lat, point_4_lon)
}

# Convert to Cartesian Coordinates
cartesian_points = {}
for point, (lat, lon) in gps_points.items():
    distance = haversine(new_origin_lat, new_origin_lon, lat, lon)
    bearing = calculate_bearing(new_origin_lat, new_origin_lon, lat, lon)
    x, y = polar_to_cartesian(distance, bearing)
    cartesian_points[point] = (x, y)

print("GPS Coordinates:", gps_points)
print("Cartesian Coordinates:", cartesian_points)
points_list = list(cartesian_points.values()) # convert the dictionary values to a list before passing them to the plot_graph 
plot_graph(points_list)