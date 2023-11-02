
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
    x = round(r * math.cos(math.radians(theta)), 2)
    y = round(r * math.sin(math.radians(theta)), 2)
    return x, y

def translate_to_origin(cartesian_polygon, origin):
    return [(x - origin[0], y - origin[1]) for x, y in cartesian_polygon]

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
    if split_point < 0 or split_point >= len(cartesian_polygon):
        return "Invalid split point"
    return cartesian_polygon[split_point + 1:] + cartesian_polygon[:split_point + 1]

# Origin point - Modify as per your requirement
origin_lat = 40.4856627992825
origin_lon = -80.33241179658118

# Path to the JSON file
file_path = 'collins_dr_62.json'

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

# Translate the coordinates so that the origin is at (0, 0)
origin_cartesian = cartesian_polygon[0]  # The new origin point is the first point in the list
cartesian_polygon_translated = translate_to_origin(cartesian_polygon, origin_cartesian)

# Plotting the polygon using the translated Cartesian coordinates
plot_graph(cartesian_polygon_translated)
