#!/usr/bin/env python3
'''
The script is designed to process a CSV file containing GPS data
1. Read the locations database to get key parameters for a site (e.g. Source file name, origin lat, lon)
2. Calculate cartesian (i.e. x and y) coordinates
3. Create inner rings based on the origianl polygon in a clockwise path; Save the rings in a .csv file to use in the Boustrophedon process and check for duplicates
4. Check for duplicates in the data
5. Determine the intersection points of the obstacles
6. Calculate the angle (i.e. theta) between points, lookahead and speed data; Save the file to the input file for pure pursuit to use

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_ll2xy_2inner_rings_v4.py

'''

import math
import matplotlib.pyplot as plt
import pandas as pd
from shapely.geometry import Polygon, MultiPolygon

import csv
import pandas as pd
import numpy as np
from matplotlib.patches import Circle

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

#from path_load_location_data import load_location_data

def convert_gps_to_cartesian(latlon_file_path, origin_lat, origin_lon):
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

    gps_data = pd.read_csv(latlon_file_path)  # Load the CSV file
    cartesian_points = []
    for index, row in gps_data.iterrows():
        distance = haversine(origin_lat, origin_lon, row['lat'], row['lng'])
        bearing = calculate_bearing(origin_lat, origin_lon, row['lat'], row['lng'])
        x = distance * math.sin(math.radians(bearing))
        y = distance * math.cos(math.radians(bearing))
        cartesian_points.append((x, y))

    gps_data['X'] = [point[0] for point in cartesian_points]  # Add the Cartesian coordinates to the dataframe
    gps_data['Y'] = [point[1] for point in cartesian_points]
    print(f"There are {len(gps_data)} records in gps_data from the .csv file")
    return gps_data

def create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name):

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
        colors = ['blue', 'green', 'red', 'purple', 'orange', 'brown']
        if len(colors) < len(paths):
            colors *= (len(paths) // len(colors) + 1)
        for index, (path, color) in enumerate(zip(paths, colors)):
            x_coords, y_coords = zip(*path)
            plt.plot(x_coords, y_coords, marker='o', color=color)
        if len(paths) > 1:
            first_path_x, first_path_y = zip(*paths[0])
            plt.text(first_path_x[-1], first_path_y[-1], '2', color='black', fontsize=10, ha='right', va='top')  # Coordinates for '2'
            second_path_x, second_path_y = zip(*paths[1])
            plt.text(second_path_x[0], second_path_y[0], '3', color='black', fontsize=10, ha='left', va='bottom') # Coordinates for '3'

        if len(paths) > 2:
            green_path_x, green_path_y = zip(*paths[1])  # Green path
            plt.text(green_path_x[-1], green_path_y[-1], '4', color='black', fontsize=10, ha='right', va='top')  # Coordinates for '4' at the end of the green path

        if len(paths) > 3:
            # Coordinates for '5' at the beginning of the red path
            red_path_x, red_path_y = zip(*paths[2])  # Red path
            plt.text(red_path_x[0], red_path_y[0], '5', color='black', fontsize=10, ha='left', va='bottom')

            # Coordinates for '6' at the end of the red path
            plt.text(red_path_x[-1], red_path_y[-1], '6', color='black', fontsize=10, ha='right', va='top')
            # Coordinates for '7' at the beginning of the purple path
            purple_path_x, purple_path_y = zip(*paths[3])  # Purple path
            plt.text(purple_path_x[0], purple_path_y[0], '7', color='black', fontsize=10, ha='left', va='bottom')

        # Plot a green dot at the start of the first path
        plt.plot(first_path_x[0], first_path_y[0], marker='o', color='green', markersize=10)

        # Plot a red dot at the end of the last path
        last_path_x, last_path_y = zip(*paths[-1])
        plt.plot(last_path_x[-1], last_path_y[-1], marker='o', color='red', markersize=10)

        plt.title('Plot of Paths')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.grid(True)
        plt.axis('equal')
        plt.show()

    def write_paths_to_csv(paths, file_name):
        with open(file_name, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Path_Index', 'X', 'Y'])  # Writing header with an additional column for Path_Index
            for index, path in enumerate(paths):
                for x, y in path:
                    writer.writerow([index, x, y])  # Writing the path index along with each coordinate pair
                #print("index:", index)

    def find_consecutive_duplicates(xy_file_name):
        duplicates = []
        with open(xy_file_name, 'r') as file:
            reader = csv.reader(file)
            header = next(reader)  # Skip the header

            # Initialize the previous row
            prev_row = None
            for i, row in enumerate(reader):
                if prev_row:
                    # Compare with the previous row
                    if prev_row[1] == row[1] and prev_row[2] == row[2]:
                        duplicates.append((i-1, i))  # Store the indices of the duplicate rows
                prev_row = row
        if duplicates:
            print("In function 'create_inner_rings', Consecutive duplicate coordinates found at row indices:")
            for dup in duplicates:
                print(dup)
        else:
            print("In function 'create_inner_rings', No consecutive duplicates found.")
        return duplicates

    def delete_lines_from_csv(xy_file_name, duplicates):
        lines_to_delete = [dup[1] for dup in duplicates]    # Extract every second value from each pair in the duplicates list
        lines_to_delete.sort(reverse=True)                  # Sort the lines to delete in descending order
        with open(xy_file_name, 'r') as file:              # Read all lines from the file and store them
            lines = file.readlines()
        for line_number in lines_to_delete:                 # Delete the specified lines
            if line_number < len(lines):                    # Check if line number is valid
                del lines[line_number]
        with open(xy_file_name, 'w') as file:              # Write the remaining lines back to the file
            file.writelines(lines)
        print("duplicates deleted from .csv file")           

    DEBUG_MODE = True  # Set to False when not debugging to avoid plotting
    # strip off the x and y coordinates from the dataframe and start working with a list
    polygon_points = list(zip(gps_data['X'], gps_data['Y']))  
    polygon = Polygon(polygon_points)   # Create polygon
    print(f"There are {len(polygon_points)} records in polygon points extracted from gps_data")

    output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_0.csv'
    with open(output_file_path, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['X', 'Y'])
        for coord in polygon.exterior.coords:
            writer.writerow(coord)
    print(f"Original polygon written to file: {output_file_path}")

    # This loop is used to create and save information about each inner ring, starting from the outermost ring and moving inward. 
    # I want the robot to beging driving with the inner most ring and work outwards in case the outer ring has issues for testing.
    paths = []
    for i in range(num_inner_rings, 0, -1):
        inner_ring = polygon.buffer(-path_size * i)

        # previous code threw a MultiPolygon error so adding this logic to check.  I'm not sure of this will fix it or not.
        if isinstance(inner_ring, MultiPolygon):
            #for polygon in inner_ring:
            for polygon in inner_ring.geoms:
                inner_points = list(polygon.exterior.coords)
        else:
            inner_points = list(inner_ring.exterior.coords)

        #inner_points = list(inner_ring.exterior.coords)
        reordered_ring = reorder_ring(ensure_clockwise(inner_points), start_point)
        paths.append(reordered_ring)
        # Save each ring to a separate CSV file
        output_file_path = f'/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_{i}.csv'
        with open(output_file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['X', 'Y'])
            for coord in reordered_ring:
                writer.writerow(coord)
        print(f"Ring {i} written to file: {output_file_path}")    
    # # save the innermost ring to be used when creating the Boustrophedon path
    # output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_innermost_ring.csv'
    # with open(output_file_path, 'w', newline='') as file:
    #     writer = csv.writer(file)
    #     writer.writerow(['X', 'Y'])
    #     for coord in reordered_ring:
    #         writer.writerow(coord)
    # print(f"Innermost ring written to file: {output_file_path}")

    # Add the original polygon to 'paths' as the last element
    original_polygon_clockwise = reorder_ring(ensure_clockwise(polygon_points)[::-1], start_point)    
    paths.append(original_polygon_clockwise)

    #xy_file_name = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_paths_coordinates.csv'
    write_paths_to_csv(paths, xy_file_name)
    print(f"Paths written to {xy_file_name}")

    duplicates = find_consecutive_duplicates(xy_file_name)
    if duplicates:
        delete_lines_from_csv(xy_file_name, duplicates)

    #print(f"There are {len(paths)} records in full ring path created from the initial polygon")
    if DEBUG_MODE:
        plot_paths(paths)
    return paths    

def update_df_with_angle_lookahead_speed(input_file_path, lookahead, speed, output_file_path):
    # Function to add angle, lookahead, and speed to the DataFrame
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

    print("calculating angles")
    df = pd.read_csv(input_file_path)  # Read CSV file into DataFrame
    print("Length of DataFrame:", len(df))
    # Calculate the angles and create a list to hold the angles
    angles = []
    for idx in range(len(df) - 1):
        x1, y1 = df.iloc[idx]['X'], df.iloc[idx]['Y']
        x2, y2 = df.iloc[idx + 1]['X'], df.iloc[idx + 1]['Y']
        angle = calculate_angle(x1, y1, x2, y2)
        angles.append(angle)
    angles.append(angles[-1])  # Handle the last point by repeating the last angle

    # Add the angle, lookahead, and speed columns to the DataFrame
    print("adding lookahead and speed")
    df['angle'] = angles
    df['lookahead'] = lookahead
    df['speed'] = speed
    df = df.drop('Path_Index', axis='columns')

    if 'Source' in df.columns:  # if there are intersections with an obstacle the file will have this column
        df = df.drop('Source', axis='columns')
 
    df.to_csv(output_file_path, index=False, header=False, sep=' ')  # no header row because the file will be read by pure_pursuit in the x, y, angle, lookahead, speed format.
    print(f"Final path published to file: {output_file_path}")
    return

def find_intersections_with_circle(xy_file_name, circle_center, radius, num_circle_points, output_file_path): 
    def calculate_circle_arc(circle_center, radius, num_points):
        segments = []
        angles = np.linspace(0, 2 * np.pi, num_points + 1)
        for i in range(num_points):
            start_angle = angles[i]
            end_angle = angles[i + 1]
            start_point = (circle_center[0] + radius * np.cos(start_angle), circle_center[1] + radius * np.sin(start_angle))
            end_point = (circle_center[0] + radius * np.cos(end_angle), circle_center[1] + radius * np.sin(end_angle))
            segments.append((start_point, end_point))
        #print("circle segments:", segments)
        return segments

    def do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end):
        def cross_product(a, b):
            return a[0] * b[1] - a[1] * b[0]

        def subtract_vectors(a, b):
            return (a[0] - b[0], a[1] - b[1])

        def is_point_on_segment(seg_start, seg_end, point):
            seg = subtract_vectors(seg_end, seg_start)
            pt_seg = subtract_vectors(point, seg_start)
            cross_prod = cross_product(seg, pt_seg)
            if abs(cross_prod) > np.finfo(float).eps:
                return False  # Point is not on the line created by the segment
            dot_prod = np.dot(seg, pt_seg)
            if dot_prod < 0 or dot_prod > np.dot(seg, seg):
                return False  # Point is outside the segment
            return True

        # Calculate vectors
        r = subtract_vectors(seg1_end, seg1_start)
        s = subtract_vectors(seg2_end, seg2_start)
        qp = subtract_vectors(seg2_start, seg1_start)

        # Calculate cross products and check if line segments are parallel
        r_cross_s = cross_product(r, s)
        qp_cross_r = cross_product(qp, r)

        # If r_cross_s is zero, line segments are parallel or collinear
        if abs(r_cross_s) < np.finfo(float).eps:
            # If qp_cross_r is also zero, line segments are collinear
            if abs(qp_cross_r) < np.finfo(float).eps:
                # Check if the segments overlap
                if is_point_on_segment(seg1_start, seg1_end, seg2_start) or is_point_on_segment(seg1_start, seg1_end, seg2_end) or \
                   is_point_on_segment(seg2_start, seg2_end, seg1_start) or is_point_on_segment(seg2_start, seg2_end, seg1_end):
                    print(seg1_start, seg1_end, seg2_start)
                    return True, (0,0)
            return False, None

        # Calculate the scalar multiples t and u for the intersection point
        t = cross_product(qp, s) / r_cross_s
        u = cross_product(qp, r) / r_cross_s

        # Check if the scalar multiples are within the range [0, 1] for both line segments
        if 0 <= t <= 1 and 0 <= u <= 1:
            # Calculate the intersection point
            intersection_x = seg1_start[0] + t * (seg1_end[0] - seg1_start[0])
            intersection_y = seg1_start[1] + t * (seg1_end[1] - seg1_start[1])
            return True, (intersection_x, intersection_y)
        return False, None

    def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points):
        # use the the circle data, plus the start and end points of the intersecting line segment to calculate the shortest path around the circle

        def adjust_angle(angle):
            """Ensure the angle is within the range [0, 2*pi]."""
            while angle < 0:
                angle += 2 * np.pi
            while angle > 2 * np.pi:
                angle -= 2 * np.pi
            return angle

        def angle_between_points(center, point):
            angle = np.arctan2(point[1] - center[1], point[0] - center[0])
            return angle

        # Determine the angles between the center of the circle and the intersection points 
        start_angle = adjust_angle(angle_between_points(circle_center, start_point))
        end_angle = adjust_angle(angle_between_points(circle_center, end_point))

        # Circumfrence of a circle is 2*Pi().  If the difference in angles > Pi() then that is more than 1/2 the circle which means it is the longer path
        angular_difference = end_angle - start_angle
        if angular_difference > np.pi:
            angular_difference -= 2 * np.pi  # Reverse the angle to get the shorter way
        elif angular_difference < -np.pi:
            angular_difference += 2 * np.pi  # Reverse the angle to get the shorter way

        # Generate points along the shortest arc using the angular difference
        arc_path = []
        for i in range(num_points + 1):
            angle = start_angle + angular_difference * i / num_points
            x = circle_center[0] + radius * np.cos(angle)
            y = circle_center[1] + radius * np.sin(angle)
            arc_path.append((x, y))
        return arc_path

    def plot_circle_and_intersections(center, radius, circle_segments, intersection_points, intersecting_segments, script_name, csv_file_path):
        plt.figure()

        # Plot the circle
        circle = Circle(center, radius, color='green', fill=False)
        plt.gca().add_patch(circle)

        # Plot the circle segments with a single legend entry
        for i, seg in enumerate(circle_segments):
            if i == 0:
                plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-', label='Circle Segments')
            else:
                plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-')

        # Plot the intersection points and label them
        if intersection_points:
            # Offset for text label
            offset = 0.1  # Adjust as needed
            for idx, point in enumerate(intersection_points):
                plt.plot(point[0], point[1], 'ro')
                plt.text(point[0] + offset, point[1] + offset, str(idx + 1), color='black', fontsize=10)

        # Plot the intersecting segments
        if intersecting_segments:
            # Similar approach as above for intersecting segments
            plt.plot([intersecting_segments[0][0][0], intersecting_segments[0][1][0]], 
                     [intersecting_segments[0][0][1], intersecting_segments[0][1][1]], 'y-', label='Intersecting Segments')
            for seg in intersecting_segments[1:]:
                plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'y-')

        plt.title(f'Script: {script_name}\nData source: {csv_file_path}')
        plt.legend()
        plt.axis('equal')

        plt.show()

    def plot_xy_coordinates(csv_file):
        # Load the data from the CSV file
        data = pd.read_csv(csv_file)
        print(f"Rows in file {csv_file} to be plotted: {len(data)}")
        # Plot the x and y coordinates
        plt.figure(figsize=(10, 6))
        plt.autoscale(enable=True, axis='both', tight=None)
        plt.scatter(data['X'], data['Y'], alpha=0.6)
        plt.title('X and Y Coordinates')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.axis('equal')  # Setting equal aspect ratio
        plt.grid(True)
        plt.show()        

    xy_file_df = pd.read_csv(xy_file_name)      # Read the CSV file
    #print(xy_file_df.head(10))
    print(f"Number of rows in DataFrame: {len(xy_file_df)}")
    circle_segments = calculate_circle_arc(circle_center, radius, num_circle_points)  # Calculate circle segments
    print(f"Number of rows in circle_segments: {len(circle_segments)}")

    # Initialize lists to store intersection data
    intersection_points = []
    intersecting_segments = []
    intersecting_index = []
    cir_seg_intersects = []
    current_path_index = None

    # Iterate through the xy_file_name data
    for i in range(len(xy_file_df) - 1):
        # xy_file_df layout - 'Path_Index' is the reference to which ring of points (i.e. a polygon) the x, y coordinate is a part of 
        #    Path_Index         X         Y
        # 0       0        -5.967672  2.907207

        seg1_start = (xy_file_df.iloc[i, 1], xy_file_df.iloc[i, 2])
        #print("seg1_start:", seg1_start)
        seg1_end = (xy_file_df.iloc[i + 1, 1], xy_file_df.iloc[i + 1, 2])
        cir_seg_ndx = 1
        path_index = xy_file_df.iloc[i, 0]   # Extract the Path_Index for the current row

        if path_index != current_path_index:
            print(f"Looking for intersects in path {path_index}")
            current_path_index = path_index

        for this_segment in circle_segments:
            seg2_start, seg2_end = this_segment
            intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
            if intersects_sw:
                intersection_points.append(intersects_pt)
                intersecting_segments.append((seg1_start, seg1_end))
                intersecting_index.append(i)
                cir_seg_intersects.append(cir_seg_ndx)
            cir_seg_ndx += 1

    # Print the results
    print("Intersection points:", intersection_points)
    print("xy_file_name rows that intersected with the circle:", intersecting_index)
    print("Circle segments with intersects:", cir_seg_intersects)
    plot_circle_and_intersections(circle_center, radius, circle_segments, intersection_points, intersecting_segments, script_name, xy_file_name)

    # Code below rebuilds the path data by using the original DataFrame, but replacing the line segments inside the obstacle/circle with the paths around the circle

    # original_data = pd.read_csv(csv_file_path)                      # Read the CSV file into a DataFrame
    # xy_file_df is the replacement for 'original_data'
    #print(xy_file_df.head(10))
    revised_path = pd.DataFrame()                                   # Initialize an empty DataFrame for revised_path

    if intersecting_index:  # Checks if the list is not empty
        # Beginning Segment
        temp_path = xy_file_df.iloc[0:intersecting_index[0]].copy()
        revised_path = pd.DataFrame()  # or however you initialize it
        temp_path['Source'] = 'temp_path_beg'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

        # Middle Segments
        qty_of_intersects = len(intersecting_index) - 2
        print("qty_of_intersects: ", qty_of_intersects)
        print("looping sequence for obstacle path")
        for i, path_index in zip(range(0, qty_of_intersects, 2), range(0, 4)):
            point1 = intersection_points[i]
            point2 = intersection_points[i + 1]
            print(i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)
            circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_circle_points)
            circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
            #circle_arc_df = circle_arc_df.iloc[::-1]  # I need to reverse the order of circle_arc_df so the points run from right to left because I'm running a clockwise pattern
            circle_arc_df['Path_Index'] = path_index
            circle_arc_df['Source'] = 'circle_arc_df'
            revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

            temp_path = xy_file_df.iloc[intersecting_index[i+1] + 1:intersecting_index[i+2]].copy()
            temp_path['Path_Index'] = path_index
            temp_path['Source'] = 'temp_path'
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)
        print("End of loop:", i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)

        # Handle End Segment
        print("Handling the end segment")
        path_index = path_index + 1
        i = i + 2
        #print(i, path_index, intersecting_index[i])
        point1 = intersection_points[i]
        point2 = intersection_points[i + 1]
        print(i, path_index, intersecting_index[i], intersecting_index[i+1], point1, point2)
        circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_circle_points)
        circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
        #circle_arc_df = circle_arc_df.iloc[::-1]  # I need to reverse the order of circle_arc_df so the points run from right to left because I'm running a clockwise pattern
        circle_arc_df['Path_Index'] = path_index
        circle_arc_df['Source'] = 'circle_arc_df_end'
        revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

        temp_path = xy_file_df.iloc[intersecting_index[i+1] + 1:].copy()
        temp_path['Path_Index'] = path_index
        temp_path['Source'] = 'end_path'
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

    #output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_continuous_obstacles.csv'
    revised_path.to_csv(output_file_path, index=False)              # Save 'revised_path' to the CSV file
    if intersecting_index:  # Checks if the list is not empty
        plot_xy_coordinates(output_file_path)
    return intersection_points, intersecting_segments, intersecting_index, cir_seg_intersects

def main():
    # read the locations database to get the name of the file where the outer polygon lat, lon, plus other info is for 'location_name'
    file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/paths_locations.csv'   # The path to the locations file
    locations_data = pd.read_csv(file_path)      # Reading the CSV file into a DataFrame both string and float data
    location_name = 'Collins_Dr_62_Site_01'
    location_row = locations_data[locations_data['location_name'] == location_name]  # Finding the specific row for the site
    if not location_row.empty:
        origin_lat = location_row['origin_lat'].iloc[0]
        origin_lon = location_row['origin_lon'].iloc[0]
        latlon_file_path = location_row['latlon_file_path'].iloc[0]
        num_inner_rings = location_row['num_inner_rings'].iloc[0]
        path_size = location_row['path_size'].iloc[0]
        start_point_x = location_row['start_point_x'].iloc[0]
        start_point_y = location_row['start_point_y'].iloc[0]
        start_point=(start_point_x, start_point_y)
        xy_file_name = location_row['xy_file_name'].iloc[0]
        #print(f"path and longitude: {location_row['latlon_file_path']}, Longitude: {location_row['origin_lon']}")
    else:
        print(f"Error: Site '{location_name}' not found in the data.")

    # calculate x, y coordinates using the lat, lon data and the origin lat, lon data that came from the locations database
    print(f"reading file: {latlon_file_path} and calculating coordinates")
    gps_data = convert_gps_to_cartesian(latlon_file_path, origin_lat, origin_lon)

    # Create inner rings based on the origianl polygon in a clockwise path; Save the rings in a .csv file to use in the Boustrophedon process and check for duplicates
    ring_path = create_inner_rings(gps_data, num_inner_rings, path_size, start_point, xy_file_name)

    # 5. Determine the intersection points of the obstacles - at the moment this can only accept one obstacle.  An enhancement is to be able to handle more obstacles.
    #additional_circle_center = (17.3, -9.1), additional_radius = 2.4)
    circle_center = (17.3, -9.1)  # use rosbag_calc_circle_radius_and_center.py to help with this
    radius = 2.4
    num_circle_points = 21
    xy_file_adjusted_for_obstacles = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_adjusted_for_obstacles.csv'
    finished_xy_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt' 
    intersections, segments, indices, circle_segments = find_intersections_with_circle(xy_file_name, circle_center, radius, num_circle_points, xy_file_adjusted_for_obstacles)

    # 6. Calculate the angle (i.e. theta) between points, add lookahead and speed data; 
    lookahead=2.5
    speed=0.75
    #speed=2.5
    if indices:  # Checks if the list is not empty
        update_df_with_angle_lookahead_speed(xy_file_adjusted_for_obstacles, lookahead, speed, finished_xy_file)
    else:
        update_df_with_angle_lookahead_speed(xy_file_name, lookahead, speed, finished_xy_file)
  
    print(f"End of main function and the overall program")

if __name__ == '__main__':
    main()
