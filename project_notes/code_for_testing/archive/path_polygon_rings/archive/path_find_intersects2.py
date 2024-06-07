#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and modify the path to avoid the obstacle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_find_intersects2.py

'''

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Function to get ring starting positions
def get_ring_starting_positions(ods_file_path):
    df = pd.read_excel(ods_file_path, sheet_name='RawInnerRings', engine='odf')
    path_index_changes = df[df['Path_Index'].diff() != 0].reset_index(drop=True)
    ring_starting_positions = [(df.iloc[0]['X'], df.iloc[0]['Y'])] + list(zip(path_index_changes['X'], path_index_changes['Y']))
    
    # Remove duplicates if the first position is repeated
    if ring_starting_positions[0] == ring_starting_positions[1]:
        ring_starting_positions.pop(1)
    
    return ring_starting_positions

# Function to update Path_Index based on starting positions
def update_path_index_based_on_starting_positions(ods_file_path, ring_starting_positions):
    df = pd.read_excel(ods_file_path, sheet_name='NewRingPath', engine='odf')
    path_index = 0

    for i in range(len(df)):
        x = df.at[i, 'X']
        y = df.at[i, 'Y']

        # Check if the current position matches any of the starting positions
        for j, (start_x, start_y) in enumerate(ring_starting_positions):
            if np.isclose(x, start_x) and np.isclose(y, start_y):
                path_index = j
                break
        
        df.at[i, 'Path_Index'] = path_index
    
    return df

# Function to check if two segments intersect
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

    r = subtract_vectors(seg1_end, seg1_start)
    s = subtract_vectors(seg2_end, seg2_start)
    qp = subtract_vectors(seg2_start, seg1_start)
    r_cross_s = cross_product(r, s)
    qp_cross_r = cross_product(qp, r)

    if abs(r_cross_s) < np.finfo(float).eps:
        if abs(qp_cross_r) < np.finfo(float).eps:
            if is_point_on_segment(seg1_start, seg1_end, seg2_start) or is_point_on_segment(seg1_start, seg1_end, seg2_end) or \
               is_point_on_segment(seg2_start, seg2_end, seg1_start) or is_point_on_segment(seg2_start, seg2_end, seg1_end):
                return True, None
        return False, None

    t = cross_product(qp, s) / r_cross_s
    u = cross_product(qp, r) / r_cross_s

    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection_x = seg1_start[0] + t * (seg1_end[0] - seg1_start[0])
        intersection_y = seg1_start[1] + t * (seg1_end[1] - seg1_start[1])
        return True, (intersection_x, intersection_y)
    return False, None

# Function to plot the circle and intersections
def plot_circle_and_intersections(center, radius, circle_segments, intersection_points, intersecting_segments):
    plt.figure()

    for i, seg in enumerate(circle_segments):
        if i == 0:
            plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-', label='Circle Segments')
        else:
            plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-')

    if intersection_points:
        offset = 0.1
        for idx, point in enumerate(intersection_points):
            plt.plot(point[0], point[1], 'ro')
            plt.text(point[0] + offset, point[1] + offset, str(idx + 1), color='black', fontsize=10)

    if intersecting_segments:
        plt.plot([intersecting_segments[0][0][0], intersecting_segments[0][1][0]], 
                 [intersecting_segments[0][0][1], intersecting_segments[0][1][1]], 'y-', label='Intersecting Segments')
        for seg in intersecting_segments[1:]:
            plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'y-')

    plt.title('Circle and Intersections')
    plt.legend()
    plt.axis('equal')
    plt.show()

# Function to calculate the shortest path around the circle
def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points):
    def adjust_angle(angle):
        while angle < 0:
            angle += 2 * np.pi
        while angle > 2 * np.pi:
            angle -= 2 * np.pi
        return angle

    def angle_between_points(center, point):
        angle = np.arctan2(point[1] - center[1], point[0] - center[0])
        return angle

    start_angle = adjust_angle(angle_between_points(circle_center, start_point))
    end_angle = adjust_angle(angle_between_points(circle_center, end_point))

    angular_difference = end_angle - start_angle
    if angular_difference > np.pi:
        angular_difference -= 2 * np.pi
    elif angular_difference < -np.pi:
        angular_difference += 2 * np.pi

    arc_path = []
    for i in range(num_points + 1):
        angle = start_angle + angular_difference * i / num_points
        x = circle_center[0] + radius * np.cos(angle)
        y = circle_center[1] + radius * np.sin(angle)
        arc_path.append((x, y))
    return arc_path

# Function to read and plot the 'NewRingPath' data
def plot_new_ring_path(ods_file_path):
    print("Plotting the new rings using data from NewRingPath sheet")
    new_ring_path = pd.read_excel(ods_file_path, sheet_name='NewRingPath', engine='odf')
    print(f"Rows in 'NewRingPath': {len(new_ring_path)}")
    plt.figure(figsize=(10, 6))
    plt.autoscale(enable=True, axis='both', tight=None)
    for path_index in new_ring_path['Path_Index'].unique():
        path_data = new_ring_path[new_ring_path['Path_Index'] == path_index]
        plt.scatter(path_data['X'], path_data['Y'], alpha=0.6, label=f'Path Index {path_index}')
    plt.title('New Ring Path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)

# Function to plot the obstacle circle from 'Obstacle 1' sheet
def plot_obstacle_circle(ods_file_path):
    obstacle_segments = pd.read_excel(ods_file_path, sheet_name='Obstacle 1', engine='odf')
    circle_segments = [((row['Start X'], row['Start Y']), (row['End X'], row['End Y'])) for _, row in obstacle_segments.iterrows()]

    for i, seg in enumerate(circle_segments):
        plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'g-', label='Obstacle Circle' if i == 0 else "")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

# Add a try-except block to read the "Obstacle 1" sheet
def read_obstacle_1_sheet(ods_file_path):
    sheet_to_read = 'Obstacle 1'
    try:
        obstacle_1_data = pd.read_excel(ods_file_path, sheet_name=sheet_to_read, engine='odf')
        print(f"Contents of {sheet_to_read} sheet:")  
        print(obstacle_1_data)
    except Exception as e:
        print(f"An error occurred while reading {sheet_to_read}: {e}")
        print(f"I suggest moving the sheet to a place other than the last place")
        print("halting program")
        sys.exit(1)

# function to find intersections
def find_intersections_with_circle(ods_file_path): 
    # Load data from the .ods file
    raw_inner_rings = pd.read_excel(ods_file_path, sheet_name='RawInnerRings', engine='odf')
    # obstacle_segments = pd.read_excel(ods_file_path, sheet_name='Obstacle 1', engine='odf')  # I had trouble so had to change approach
    obstacle_segments = read_obstacle_1_sheet(ods_file_path)
    print(f"after obstacle 1 read.  Obstacle segments: {obstacle_segments}")
    circle_segments = [((row['Start X'], row['Start Y']), (row['End X'], row['End Y'])) for _, row in obstacle_segments.iterrows()]
    print("after circle segments captured")
    circle_center = (np.mean([seg[0][0] for seg in circle_segments]), np.mean([seg[0][1] for seg in circle_segments]))
    radius = np.mean([np.sqrt((seg[0][0] - circle_center[0])**2 + (seg[0][1] - circle_center[1])**2) for seg in circle_segments])

    intersection_points = []
    intersecting_segments = []
    intersecting_index = []
    cir_seg_intersects = []
    current_path_index = None
    print("Loop through each segment of the raw inner rings")
    # Loop through each segment of the raw inner rings
    for i in range(len(raw_inner_rings) - 1):
        # Define the start and end points of the current segment from the raw inner rings because my input is a list of waypoints
        seg1_start = (raw_inner_rings.iloc[i, 1], raw_inner_rings.iloc[i, 2])
        seg1_end = (raw_inner_rings.iloc[i + 1, 1], raw_inner_rings.iloc[i + 1, 2])
        
        # Extract the path index for the current segment
        path_index = raw_inner_rings.iloc[i, 0]
        
        # Initialize the index for circle segments
        cir_seg_ndx = 1

        # Check if we are moving to a new path
        if path_index != current_path_index:
            print(f"Looking for intersects in ring {path_index}")
            current_path_index = path_index

        # Loop through each segment of the circle
        for this_segment in circle_segments:
            # Define the start and end points of the current segment from the circle
            seg2_start, seg2_end = this_segment
            
            # Check if the segments intersect
            intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
            
            if intersects_sw:
                # Ensure the intersection point is not None
                if intersects_pt:
                    # Avoid adding the placeholder (0, 0) as an intersection point
                    if intersects_pt != (0, 0):
                        # Avoid adding duplicate intersection points
                        if intersects_pt not in intersection_points:
                            print(f"Intersection found between path segment {seg1_start}-{seg1_end} and circle segment {seg2_start}-{seg2_end}")
                            
                            # Add the intersection point to the list
                            intersection_points.append(intersects_pt)
                            print(f"Intersection point is: {intersects_pt}")
                            
                            # Add the intersecting segment to the list
                            intersecting_segments.append((seg1_start, seg1_end))
                            
                            # Add the index of the intersecting segment to the list
                            intersecting_index.append(i)
                            print(f"The index of the intersecting segment (aka row in RawInnerRings) is: {i+2}")
                            
                            # Add the circle segment index to the list
                            cir_seg_intersects.append(cir_seg_ndx)
                            print(f"The index of the circle segment (aka row in Obstacle 1) is: {cir_seg_ndx +1}")
            
            # Increment the circle segment index
            cir_seg_ndx += 1

    # Print the intersection points for debugging purposes
    print("Completed Intersection points:", intersection_points)

    print("Rows that intersected with the circle:", intersecting_index)
    print("Circle segments with intersects:", cir_seg_intersects)

    plot_circle_and_intersections(None, None, circle_segments, intersection_points, intersecting_segments)

    # Initialize an empty DataFrame to store the revised path
    revised_path = pd.DataFrame()

    # Check if there are any intersections
    if intersecting_index:
        # Get the current path index from the raw inner rings
        current_path_index = raw_inner_rings.iloc[0, 0]
        
        # Extract the initial segment of the path up to the first intersection
        temp_path = raw_inner_rings.iloc[0:intersecting_index[0]].copy()
        temp_path['Source'] = 'path_segment_0'
        
        # Add the initial segment to the revised path DataFrame
        revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

        # Get the total number of intersections
        qty_of_intersects = len(intersecting_index)

        # Initialize a counter for unique source values
        source_counter = 0

        # Iterate over the intersections in pairs (two at a time)
        for i in range(0, qty_of_intersects - 1, 2):
            # Get the intersection points
            point1 = intersection_points[i]
            point2 = intersection_points[i + 1]
            
            # Use the same Path_Index for the new segment around the obstacle
            path_index = raw_inner_rings.iloc[intersecting_index[i], 0]

            # Calculate the shortest path around the obstacle between the intersection points
            circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, 20)
            
            # Create a DataFrame for the circle arc
            circle_arc_df = pd.DataFrame(circle_arc, columns=['X', 'Y'])
            circle_arc_df['Path_Index'] = path_index
            circle_arc_df['Source'] = f'circle_arc_df_{source_counter}'
            
            # Add the circle arc to the revised path DataFrame
            revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

            # Increment the source counter for uniqueness
            source_counter += 1

            # Determine the next segment of the path after the second intersection
            if i + 2 < qty_of_intersects:
                # If there are more intersections, extract the segment up to the next intersection
                temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:intersecting_index[i + 2]].copy()
            else:
                # If this is the last intersection, extract the remaining path
                temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:].copy()
            
            # Use the same Path_Index for this segment
            temp_path['Path_Index'] = path_index
            temp_path['Source'] = f'path_segment_{source_counter}'
            
            # Add this segment to the revised path DataFrame
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)

            # Increment the source counter for uniqueness
            source_counter += 1

        # Handle the case where there's an odd number of intersections (final segment)
        print(f"Checking odd number of intersections; i: {i+2}; qty_of_intersects: {qty_of_intersects}")
        if ((i + 2) < qty_of_intersects):
            print("Warning - unexpected odd number of intersections")
            # Extract the remaining segment of the path after the last intersection
            temp_path = raw_inner_rings.iloc[intersecting_index[i + 1] + 1:].copy()
            
            # Use the same Path_Index for this segment
            temp_path['Path_Index'] = raw_inner_rings.iloc[intersecting_index[i + 1], 0]
            temp_path['Source'] = f'end_path_{source_counter}'
            
            # Add this segment to the revised path DataFrame
            revised_path = pd.concat([revised_path, temp_path], ignore_index=True)


    # Read existing sheets
    with pd.ExcelFile(ods_file_path, engine='odf') as xls:
        sheet_names = xls.sheet_names
        sheet_data = {sheet: pd.read_excel(xls, sheet_name=sheet) for sheet in sheet_names}

    # Add the new sheet data
    print("Adding data to sheet NewRingPath")
    sheet_data['NewRingPath'] = revised_path

    # Write all sheets back to the .ods file
    with pd.ExcelWriter(ods_file_path, engine='odf') as writer:
        for sheet_name, df in sheet_data.items():
            df.to_excel(writer, sheet_name=sheet_name, index=False)

    return intersection_points, intersecting_segments, intersecting_index, cir_seg_intersects

# Main Function
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
find_intersections_with_circle(ods_file_path)

print("getting starting positions")
ring_starting_positions = get_ring_starting_positions(ods_file_path)
print("Starting positions:", ring_starting_positions)
print("updating Path_Index")
updated_new_ring_path = update_path_index_based_on_starting_positions(ods_file_path, ring_starting_positions)
# Read existing sheets
with pd.ExcelFile(ods_file_path, engine='odf') as xls:
    sheet_names = xls.sheet_names
    sheet_data = {sheet: pd.read_excel(xls, sheet_name=sheet) for sheet in sheet_names}

# Update the NewRingPath sheet data
sheet_data['NewRingPath'] = updated_new_ring_path

# Write all sheets back to the .ods file
with pd.ExcelWriter(ods_file_path, engine='odf') as writer:
    for sheet_name, df in sheet_data.items():
        df.to_excel(writer, sheet_name=sheet_name, index=False)

print("Path_Index updated and saved to 'NewRingPath' sheet.")
plot_new_ring_path(ods_file_path)

plot_obstacle_circle(ods_file_path)
plt.show()
