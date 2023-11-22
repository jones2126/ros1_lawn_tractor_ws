
import math

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def calculate_total_length(points):
    # Calculate the total path length
    total_length = 0.0
    for i in range(len(points) - 1):
        total_length += calculate_distance(points[i][0], points[i][1], points[i+1][0], points[i+1][1])
    # Add distance from the last point to the first to close the path
    total_length += calculate_distance(points[-1][0], points[-1][1], points[0][0], points[0][1])
    return total_length

def calculate_polygon_area(points):
    '''
    Calculates the area of a polygon is based on the shoelace formula, which is a mathematical algorithm to determine the area of a 
    simple polygon whose vertices are described by ordered pairs in the plane.  For the shoelace formula to work correctly, the polygon 
    must be closed; that is, the first and the last vertex must be the same. This is because the formula works by summing the 
    cross-products of the coordinates of adjacent vertices, including the product of the last and first vertices. If the polygon is 
    not closed (i.e., the first and last points are not the same), the calculation will not include the area component defined by the 
    line segment connecting the last point to the first point, and the resulting area will be incorrect.
    '''
    if points[0] != points[-1]:
        print("The polygon is not closed.  Unstable results")
    # Calculate the area of a polygon
    n = len(points)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += points[i][0] * points[j][1]
        area -= points[j][0] * points[i][1]
    area = abs(area) / 2.0
    return area

def create_polygon(origin, sides, area_meters_sq):
    '''
    This process creates a list of points (x, y) that form the vertices of the regular polygon, with the first and last point being the 
    same to close the loop of the polygon.

    For a regular polygon with 'n' sides and side length 's', the area 'A' can be calculated using the formula:
    Area (A) = (1/4) * n * s^2 * cot(PI/n), but in this function area will be provided as an input.

    From this formula, we can solve for the side length 's':
    side_length (s) = math.sqrt((4 * area_meters_sq * math.tan(math.pi / sides)) / sides)

    Once we have the side length, we can calculate the radius 'R' of the circumscribed circle (which will help us place the points) using the formula:
    Radius (R) = s / (2 * sin(PI/n))

    With the radius 'R' and the center coordinates (x0, y0), we can calculate the vertices of the polygon. For each vertex 'i' (where i ranges 
    from 0 to n-1), the coordinates (x, y) can be calculated using the following transformations:
    x = x0 + R * cos(theta)
    y = y0 + R * sin(theta)

    Here, 'theta' is the angle for the current vertex. It starts from the top (negative y-axis) and moves counterclockwise around the 
    polygon. The angle 'theta' can be calculated for each vertex as:
    Theta (theta) = 2 * PI * i / n - PI/2
    '''

    # Calculate the side length needed for the desired area of a regular polygon
    angle = math.pi / sides
    tan_angle = math.tan(angle)  # tangent of angle
    side_length = math.sqrt((4 * area_meters_sq * tan_angle) / sides)
    radius = side_length / (2 * math.sin(angle))      # Calculate the radius of the circumscribed circle

    # Calculate the vertices of the polygon
    polygon_points = []
    for i in range(sides):
        theta = (2 * math.pi * i / sides) - (math.pi / 2)  # Start from the top (270 degrees)
        x = origin[0] + radius * math.cos(theta)
        y = origin[1] + radius * math.sin(theta)
        x = round(x, 2)
        y = round(y, 2)
        polygon_points.append((x, y))

    # Close the polygon by repeating the first point
    polygon_points.append(polygon_points[0])

    return polygon_points, side_length

def plot_graph(points, script_name, data_source, keep_plot_open='N', annotate_points='N'):
    #This function creates a PNG image file in the '/home/tractor/Pictures' directory with a name following the 'PlotImage{date_time}.png' format. 

    import matplotlib.pyplot as plt
    from datetime import datetime
    import os

    x_coords = [x for x, y in points]  # Extracting x and y coordinates from the points list
    y_coords = [y for x, y in points]
    plt.scatter(x_coords, y_coords, color='red')  # Plotting the points
    
    # Plotting line segments individually
    num_points = len(points)
    for i in range(num_points - 1):
        plt.plot([points[i][0], points[i + 1][0]], [points[i][1], points[i + 1][1]], 'b-')

    # Annotating the points
    # for i, (x, y) in enumerate(points):
    #     plt.annotate(f'Point {i + 1}', (x, y), textcoords="offset points", xytext=(0,10), ha='center')

    # Annotating the points if annotate_points is 'Y'
    if annotate_points == 'Y':
        for i, (x, y) in enumerate(points):
            plt.annotate(f'Point {i + 1}', (x, y), textcoords="offset points", xytext=(0,10), ha='center')

    plt.axis('equal')

    # Adding the script name that called the function as a title
    plt.title(f'Script: {script_name}\nData source: {data_source}')
    
    # Generating the filename with the current datetime
    current_time = datetime.now()
    formatted_time = current_time.strftime('%Y-%m-%d-%H-%M-%S')
    filename = f'PlotImage{formatted_time}.png'
    filepath = os.path.join('/home/tractor/Pictures', filename)
    
    # Saving the plot to the file
    plt.savefig(filepath)
    
    # Print the path and filename
    print(f"Plot image saved to: {filepath}")
    
    # Decide whether to display the plot based on keep_plot_open value
    if keep_plot_open == 'Y':
        plt.show()
    else:
        plt.close()  # Close the figure to prevent it from showing


def meters_to_feet(meters):
    feet = meters * 3.28084
    return feet
def square_meters_to_acres(square_meters):
    acres = square_meters / 4046.85642
    return acres
def calculate_speed(meters, seconds):
    if seconds == 0:
        return "Duration cannot be zero."
    speed_m_s = meters / seconds   # Speed in meters per second
    meters_in_a_mile = 1609.34
    seconds_in_an_hour = 3600
    speed_mph = (meters / meters_in_a_mile) * (seconds_in_an_hour / seconds)  # Speed in miles per hour
    return speed_mph, speed_m_s    

def get_start_end_times(bag_path):

    """
    Extracts the wall clock date and time of the first and last records in a ROS bag file.

    Parameters:
    - bag_path: The file path to the ROS bag file.

    Returns:
    - A tuple of datetime objects representing the start and end times.

    Example usage:
        bag_path = 'path_to_your_bag_file.bag'
        start_time, end_time = get_start_end_times(bag_path)
        print("Start time:", start_time)
        print("End time:", end_time)  
    """
    print("Initiating rosbag utility get_start_end_times")
    import rosbag
    from datetime import datetime    
    with rosbag.Bag(bag_path, 'r') as bag:
        # Get the first message timestamp
        _, _, first_time = next(bag.read_messages())
        start_time = datetime.fromtimestamp(first_time.to_sec())
        print("Start time collected; Iterating over file to get end time...")
        # Initialize last time as the first time initially
        last_time = first_time

        # Iterate through the bag file to get to the last message
        for _, _, t in bag.read_messages():
            last_time = t

        end_time = datetime.fromtimestamp(last_time.to_sec())

        return start_time, end_time


def build_points_list(file_path, x_column_title, y_column_title):
    """
    The function will read a CSV file and build a list of points from the specified x and y columns which will be returned as a 
    list of tuples, where each tuple represents a point with its x and y coordinates. 

    Example usage:
        points, num_points = build_points_list('path_to_csv.csv', 'X', 'Y')

        To use this function, you would replace 'path_to_csv.csv', 'X', and 'Y' with the actual file path and column names in file        
  
    """    
    print("Initiating function build_points_list")
    import pandas as pd
    data = pd.read_csv(file_path)
    x_coords = data[x_column_title].tolist()
    y_coords = data[y_column_title].tolist()
    points = list(zip(x_coords, y_coords))
    num_points = len(points)
    return points, num_points

def load_locations_db(file_path):
    import csv
    locations_data = []
    with open(file_path, 'r') as csvfile:
        csv_reader = csv.reader(csvfile, delimiter=',')
        for row in csv_reader:
            location_name = row[0].strip()
            origin_lat = float(row[1].strip()) 
            origin_lon = float(row[2].strip())
            csv_file_path = row[3].strip()
            locations_data.append({
                'location_name': location_name,
                'origin_lat': origin_lat,
                'origin_lon': origin_lon,
                'csv_file_path': csv_file_path
            })
    return locations_data



# Calculate distance between two points using the Haversine formula
def calculate_latlon_distance(lat1, lon1, lat2, lon2):
    import math
    # Radius of the Earth in km
    R = 6371.0

    # Convert coordinates from degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Difference in coordinates
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers
    distance = R * c
    return distance

def find_southernmost_point_with_index(points):
    """
    This function calculates the southernmost point which will have the smallest latitude.  The min() function compares 
    points based on their latitude 'point[0]' and returns 'southernmost' and also the location of the point in the original
    list.
    """    
    southernmost = min(points, key=lambda point: point[0])
    southernmost_index = points.index(southernmost)
    return southernmost, southernmost_index

def calculate_ROS_angle(x1, y1, x2, y2):
    '''
    calculates the directional angle with respect to the positive X-axis. This is in line with the ROS REP 103 standard, where an angle 
    of 0 radians corresponds to movement directly along the positive X-axis and approximately 1.57 radians corresponds to movement 
    directly along the positive Y-axis.
    '''    
    angle = math.atan2(y2 - y1, x2 - x1)
    if angle < 0:  # this ensures I get a value between 0 and 2Pi()
        angle += 2 * math.pi
    return angle

def calculate_bearing(lat1, lon1, lat2, lon2):
    d_lon = math.radians(lon2 - lon1)
    x = math.sin(d_lon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(d_lon))
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing    

def calculate_slope(x1, y1, x2, y2):
    # Calculate the slope of the line between two points (i.e. delta_y / delta_x)
    # for lat lon the input is (lat1, lon1, lat2, lon2)
    delta_y = x2 - x1
    delta_x = y2 - y1
    # Avoid division by zero for vertical lines
    if delta_x == 0:
        return float('inf')  # Infinite slope
    else:
        return delta_y / delta_x  


def write_to_csv(data, headers, file_path):
    """
    Writes data to a CSV file.

    :param data: A list of tuples containing the data to be written.
    :param headers: A list of strings representing the header of the CSV.
    :param file_path: The path where the CSV file will be saved.
    :return: The path of the output CSV file.
    """
    import csv
    with open(file_path, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(headers)  # Write the header
        csvwriter.writerows(data)  # Write the data
    print(f"The .csv file has been written to: {file_path}")

def slice_points(points, start_index, end_index):
    """
    Returns a new list containing the points from the start index to the end index, inclusive.

    :param points: The original list of points.
    :param start_index: The starting index for the slice.
    :param end_index: The ending index for the slice.
    :return: A new list with the sliced points.
    """
    # Slicing the list to include the end index
    return points[start_index:end_index + 1]

# def convert_gps_to_cartesian(csv_file_path, origin_lat, origin_lon):
#     gps_data = pd.read_csv(csv_file_path)  # Load the CSV file
#     cartesian_points = []
#     for index, row in gps_data.iterrows():
#         distance = haversine(origin_lat, origin_lon, row['lat'], row['lng'])
#         bearing = calculate_bearing(origin_lat, origin_lon, row['lat'], row['lng'])
#         x = distance * math.sin(math.radians(bearing))
#         y = distance * math.cos(math.radians(bearing))
#         cartesian_points.append((x, y))

#     gps_data['X'] = [point[0] for point in cartesian_points]  # Add the Cartesian coordinates to the dataframe
#     gps_data['Y'] = [point[1] for point in cartesian_points]
#     return gps_data

def convert_gps_to_cartesian(csv_file_path_or_df, origin_lat, origin_lon):
    import pandas as pd
    import math

    # Determine the type of the csv_file_path_or_df and act accordingly
    # Check if the input is a DataFrame, list of tuples, or list of lists
    if isinstance(csv_file_path_or_df, pd.DataFrame):
        gps_data = csv_file_path_or_df
    elif isinstance(csv_file_path_or_df, list):
        # Check if the first element is a tuple or list with two elements (lat, lon)
        if all(isinstance(point, (tuple, list)) and len(point) == 2 for point in csv_file_path_or_df):
            gps_data = pd.DataFrame(csv_file_path_or_df, columns=['lat', 'lng'])
        else:
            raise ValueError("List must contain tuples or lists with two elements (lat, lon) each.")
    else:
        raise ValueError("Input must be a DataFrame or a list of (lat, lon) tuples/lists.")


    #gps_data = pd.read_csv(csv_file_path)  # Load the CSV file
    x_coords = []
    y_coords = []
    for index, row in gps_data.iterrows():
        distance = haversine(origin_lat, origin_lon, row['lat'], row['lng'])
        bearing = calculate_bearing(origin_lat, origin_lon, row['lat'], row['lng'])
        x = distance * math.sin(math.radians(bearing))
        y = distance * math.cos(math.radians(bearing))
        x_coords.append(x)
        y_coords.append(y)

    gps_data['X'] = x_coords  # Add the X coordinates to the dataframe
    gps_data['Y'] = y_coords  # Add the Y coordinates to the dataframe
    return gps_data 

def haversine(lat1, lon1, lat2, lon2):
    R = 6378137  # Radius of Earth in meters
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance    