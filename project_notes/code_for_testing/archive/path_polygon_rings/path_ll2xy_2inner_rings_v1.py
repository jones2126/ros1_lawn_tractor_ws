#!/usr/bin/env python3
'''
The script is designed to process a CSV file containing GPS data
1. Read the locations database to get key parameters for a site (e.g. Source file name, origin lat, lon)
2. Calculate cartesian coordinates
3. Create inner rings based on the origianl polygon in a clockwise path; Save the innermost ring to feed to the Boustrophedon process
4. Check for duplicates in the data
5. Determine the intersection points of the obstacles
6. Calculate the angle (i.e. theta) between points
7. If this will be the final path, add lookahead and speed data; If its not the final path (e.g. you might want to add the boustrophedon path first) then you can 
   skip this step or you can save this file and then merge them later.
8. Save the file
9. If this will be the final path, copy the output to the input file for pure pursuit to use
10. Plot the path coordinates using matplotlib. 

There is untested code that is commented out that would add a column to the .csv file indicating whether the point should
be plotted.  The idea was to reduce the number of plotted points based on a minimum distance.


$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_ll2xy_2inner_rings_v1.py

'''

import math
import matplotlib.pyplot as plt
import pandas as pd
from shapely.geometry import Polygon
#from path_load_location_data import load_location_data

def convert_gps_to_cartesian(csv_file_path, origin_lat, origin_lon):
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

    gps_data = pd.read_csv(csv_file_path)  # Load the CSV file
    cartesian_points = []
    for index, row in gps_data.iterrows():
        distance = haversine(origin_lat, origin_lon, row['lat'], row['lng'])
        bearing = calculate_bearing(origin_lat, origin_lon, row['lat'], row['lng'])
        x = distance * math.sin(math.radians(bearing))
        y = distance * math.cos(math.radians(bearing))
        cartesian_points.append((x, y))

    gps_data['X'] = [point[0] for point in cartesian_points]  # Add the Cartesian coordinates to the dataframe
    gps_data['Y'] = [point[1] for point in cartesian_points]
    #print(gps_data.head(10))
    print(f"There are {len(gps_data)} records in gps_data from the .csv file")
    return gps_data

def create_inner_rings(gps_data, num_inner_rings, path_size, start_point):
    """
    Creates a specified number of inner rings within a given polygon.

    Parameters:
    polygon_points (list): List of points (tuples) that make up the polygon.
    num_inner_rings (int): Number of inner rings to generate.
    path_size (float): Gap size between each inner ring and the original polygon.
    start_point (tuple): Starting point for reordering the ring coordinates.

    Returns:
    list: A list of paths for the inner rings and the original polygon.
    """

    # Function to reorder a ring so that it starts near a the specified point 'start_point'
    def reorder_ring(ring, start_point):
        closest_index = min(range(len(ring)), key=lambda i: (ring[i][0] - start_point[0])**2 + (ring[i][1] - start_point[1])**2)  # Find the point in the ring closest to the start_point
        reordered_ring = ring[closest_index:] + ring[:closest_index]   # Reorder the ring to start from the closest point
        return reordered_ring        

    # Ensure the inner rings and original polygon are in clockwise order
    def ensure_clockwise(polygon):
        if Polygon(polygon).area < 0:   # If area is negative, the polygon is clockwise
            return polygon[::-1]        # Reverse to make it clockwise
        return polygon

    def plot_paths(paths):
        plt.figure(figsize=(10, 8))

        # Define a list of colors for the paths
        colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown']
        # Ensure there are enough colors for the number of paths
        if len(colors) < len(paths):
            # If not, repeat the color list
            colors *= (len(paths) // len(colors) + 1)  # repeat the list colors to cover the number of elements in paths by multiplying the list colors by the number calculated.  

        for path, color in zip(paths, colors):
            x_coords, y_coords = zip(*path)  # Separate the x and y coordinates
            plt.plot(x_coords, y_coords, marker='o', color=color)  # Plot each path with a specific color

        # Plot a green dot at the start of the first path
        first_path_x, first_path_y = zip(*paths[0])
        plt.plot(first_path_x[0], first_path_y[0], marker='o', color='green', markersize=10)

        # Plot a red dot at the end of the last path
        last_path_x, last_path_y = zip(*paths[-1])
        plt.plot(last_path_x[-1], last_path_y[-1], marker='o', color='red', markersize=10)

        plt.title('Plot of Paths')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.axis('equal')  # Equal scaling of x and y axes
        plt.show()        

    DEBUG_MODE = True  # Set to False when not debugging to avoid plotting
    # strip off the x and y coordinates from the dataframe and start working with a list
    polygon_points = list(zip(gps_data['X'], gps_data['Y']))  
    polygon = Polygon(polygon_points)   # Create polygon
    print(f"There are {len(polygon_points)} records in polygon points extracted from gps_data")

    # Create inner rings in reverse order because we want the robot to start with the inner most ring and work outwards in case the outer ring has issues for testing
    paths = []
    for i in range(num_inner_rings, 0, -1):
        inner_ring = polygon.buffer(-path_size * i)
        inner_points = list(inner_ring.exterior.coords)
        reordered_ring = reorder_ring(ensure_clockwise(inner_points), start_point)
        paths.append(reordered_ring)

    # Add the original polygon to 'paths' as the last element
    original_polygon_clockwise = reorder_ring(ensure_clockwise(polygon_points)[::-1], start_point)    
    paths.append(original_polygon_clockwise)


    # # Create inner rings
    # paths = []
    # for i in range(1, num_inner_rings + 1):
    #     inner_ring = polygon.buffer(-path_size * i)
    #     inner_points = list(inner_ring.exterior.coords)
    #     reordered_ring = reorder_ring(ensure_clockwise(inner_points), start_point)
    #     paths.append(reordered_ring)
    #     print(f"Number of points in inner ring {i}: {len(reordered_ring)}, Total paths size: {len(paths)}")        

    # # Add the original polygon to 'paths' because I want to run this last
    # original_polygon_clockwise = reorder_ring(ensure_clockwise(polygon_points)[::-1], start_point)    
    # paths.append(original_polygon_clockwise)


    print(f"There are {len(paths)} records in full ring path created from the initial polygon")
    if DEBUG_MODE:
        plot_paths(paths)
    return paths    

# def plot_cartesian_points(ring_path):
#     x_coords = [point[0] for point in ring_path]  # Extract X coordinates
#     y_coords = [point[1] for point in ring_path]  # Extract Y coordinates    

#     # Plot the X and Y coordinates
#     plt.figure(figsize=(10, 8))
#     plt.scatter(x_coords, y_coords, c='red', label='GPS Points')
#     plt.plot(x_coords, y_coords, 'b-', label='Path')
#     plt.title('Plot of Cartesian Coordinates')
#     plt.xlabel('X Coordinate (meters)')
#     plt.ylabel('Y Coordinate (meters)')
#     plt.legend()
#     plt.axis('equal')  # Ensure equal scaling on both axes
#     plt.grid(True)
#     plt.show()


#Modify the plot function to plot only the points with 'to_plot' flag as 'y'

# def plot_cartesian_points(gps_data):
#     Filter points to plot
#     points_to_plot = gps_data[gps_data['to_plot'] == 'y']
#     x_coords = points_to_plot['X'].to_numpy()
#     y_coords = points_to_plot['Y'].to_numpy()
#     Plot the X and Y coordinates
#     plt.figure(figsize=(10, 8))
#     plt.scatter(x_coords, y_coords, c='red', label='GPS Points')
#     plt.plot(x_coords, y_coords, 'b-', label='Path')
#     plt.title('Plot of Cartesian Coordinates')
#     plt.xlabel('X Coordinate (meters)')
#     plt.ylabel('Y Coordinate (meters)')
#     plt.legend()
#     plt.axis('equal')  Ensure equal scaling on both axes
#     plt.grid(True)
#     plt.show()


def calculate_angle(x1, y1, x2, y2):
    '''
    calculates the directional angle with respect to the positive X-axis. This is in line with the ROS REP 103 standard, where an angle 
    of 0 radians corresponds to movement directly along the positive X-axis and approximately 1.57 radians corresponds to movement 
    directly along the positive Y-axis.
    '''    
    angle = math.atan2(y2 - y1, x2 - x1)
    if angle < 0:  # this ensures I get a value between 0 and 2Pi()
        angle += 2 * math.pi
    return angle

# Function to add angle, lookahead, and speed to the DataFrame
def update_df_with_angle_lookahead_speed(df, lookahead, speed):
    # Calculate the angles and create a list to hold the angles
    angles = []
    for idx in range(len(df) - 1):
        x1, y1 = df.iloc[idx]['X'], df.iloc[idx]['Y']
        x2, y2 = df.iloc[idx + 1]['X'], df.iloc[idx + 1]['Y']
        angle = calculate_angle(x1, y1, x2, y2)
        angles.append(angle)
    angles.append(angles[-1])  # Handle the last point by repeating the last angle

    # Add the angle, lookahead, and speed columns to the DataFrame
    df['angle'] = angles
    df['lookahead'] = lookahead
    df['speed'] = speed
    return df

def main():
    # read the locations database to get the name of the file where the outer polygon lat, lon, plus other info is for 'location_name'
    file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/paths_locations.csv'   # The path to the locations file
    locations_data = pd.read_csv(file_path)      # Reading the CSV file into a DataFrame both string and float data
    #locations_data.columns = locations_data.columns.str.strip() # In case there are any spaces in my header file
    print(locations_data)
    location_name = 'Collins_Dr_62_Site_01'
    location_row = locations_data[locations_data['location_name'] == location_name]  # Finding the specific row for the site
    csv_file_path = location_row['csv_file_path'].iloc[0]
    origin_lat = location_row['origin_lat'].iloc[0]
    print(f"paths_locations.csv file origin_lat: ('{origin_lat}')")
    print(f"Data type of origin_lat: {type(origin_lat)}")
    if not location_row.empty:
        origin_lat = location_row['origin_lat'].iloc[0]
        origin_lon = location_row['origin_lon'].iloc[0]
        csv_file_path = location_row['csv_file_path'].iloc[0]
        num_inner_rings = location_row['num_inner_rings'].iloc[0]
        path_size = location_row['path_size'].iloc[0]
        start_point_x = location_row['start_point_x'].iloc[0]
        start_point_y = location_row['start_point_y'].iloc[0]
        start_point=(start_point_x, start_point_y)
        print(f"path and longitude: {location_row['csv_file_path']}, Longitude: {location_row['origin_lon']}")
    else:
        print(f"Error: Site '{location_name}' not found in the data.")

    # calculate x, y coordinates using the lat, lon data and the origin lat, lon data that came from the locations database
    print(f"reading file: {csv_file_path} and calculating x and y coordinates...")
    gps_data = convert_gps_to_cartesian(csv_file_path, origin_lat, origin_lon)

    # Create inner rings based on the origianl polygon in a clockwise path 

    ring_path = create_inner_rings(gps_data, num_inner_rings, path_size, start_point)


# Save the innermost ring to feed to the Boustrophedon process
# 4. Check for duplicates in the data
# 5. Determine the intersection points of the obstacles
# 6. Calculate the angle (i.e. theta) between points
# 7. If this will be the final path, add lookahead and speed data; If its not the final path (e.g. you might want to add the boustrophedon path first) then you can 
#    skip this step or you can save this file and then merge them later.
# 8. Save the file
# 9. If this will be the final path, copy the output to the input file for pure pursuit to use
# 10. Plot the path coordinates using matplotlib. 


    #plot_cartesian_points(ring_path)

    # # 
    # print(f"Adding angles based on x and y coordinate...")    
    # #gps_data = update_df_with_angle_lookahead_speed(gps_data, lookahead=2.5, speed=0.75)  
    # print(f"Re-writing to the .csv file: {csv_file_path}")
    # #gps_data.to_csv(csv_file_path, index=False)
    # #output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt' 
    # print(f"Publishing final path to file: {csv_file_path}") 
    # #output_to_pure_pursuit(gps_data, output_file_path)    
    print(f"End of main function")

def output_to_pure_pursuit(df, output_file_path):
    """
    Extracts the X, Y, angle, lookahead, and speed data from the DataFrame and writes to a text file.
    
    :param df: DataFrame containing the path data with X, Y, angle, lookahead, and speed.
    :param output_file_path: Path to the output text file.
    """
    with open(output_file_path, 'w') as file:
        for index, row in df.iterrows():
            file.write(f"{row['X']} {row['Y']} {row['angle']} {row['lookahead']} {row['speed']}\n")

if __name__ == '__main__':
    main()
