# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/path_obstacle_handling_v8.py
# /home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path_duplicates_removed.csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math
import pandas as pd
import csv

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

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

def plot_path_and_obstacle(circle_center, radius, full_path):
    print("circle_center:", circle_center)
    fig, ax = plt.subplots()

    # Plot the circle (obstacle)
    circle = plt.Circle(circle_center, radius, color='blue', fill=False)
    ax.add_artist(circle)

    # Plot the full path
    # Extracting x and y coordinates from the path points
    x_coords = [point[0] for point in full_path]
    y_coords = [point[1] for point in full_path]
    ax.plot(x_coords, y_coords, 'r-', label='Path')

    # Mark start and end points
    ax.plot(full_path[0][0], full_path[0][1], 'go', label='Start')
    ax.plot(full_path[-1][0], full_path[-1][1], 'mo', label='End')

    # Calculate and set plot limits with padding using the circle size as the driver
    padding = 1  # Adjust the padding as needed
    ax.set_xlim(circle_center[0] - radius - padding, circle_center[0] + radius + padding)
    ax.set_ylim(circle_center[1] - radius - padding, circle_center[1] + radius + padding)


    # Make axes equal to maintain aspect ratio
    #ax.set_aspect('equal', adjustable='box')
    ax.set_aspect('equal')    

    # Adding labels and legend
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.legend()

    # Show plot
    plt.show()

def distance_between_points(point1, point2):
    """Calculate the distance between two points."""
    distance = math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
    #print("point1", point1, "point2", point2, "distance", distance)
    return distance

def find_duplicate_coordinates(df):
    # List to store the indices of duplicate rows
    duplicate_indices = []

    # Iterate through the DataFrame
    for i in range(len(df) - 1):
        # Compare current row with the next row
        if df.iloc[i]['x'] == df.iloc[i + 1]['x'] and df.iloc[i]['y'] == df.iloc[i + 1]['y']:
            duplicate_indices.append(i)
    return duplicate_indices

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

def calculate_circle_arc(circle_center, radius, num_points):
    segments = []
    angles = np.linspace(0, 2 * np.pi, num_points + 1)
    for i in range(num_points):
        start_angle = angles[i]
        end_angle = angles[i + 1]
        start_point = (circle_center[0] + radius * np.cos(start_angle), circle_center[1] + radius * np.sin(start_angle))
        end_point = (circle_center[0] + radius * np.cos(end_angle), circle_center[1] + radius * np.sin(end_angle))
        segments.append((start_point, end_point))
    # print("circle segments:", segments)
    return segments

# use the the circle data, plus the start and end points of the intersecting line segment to calculate the shortest path around the circle
def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points):
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


# Path to the CSV file
csv_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path_duplicates_removed.csv'
df = pd.read_csv(csv_file_path)  # Read the CSV file

duplicates = find_duplicate_coordinates(df)
print("if there are duplicates, delete these lines (+2) from the data and restart: ", duplicates)

# # we need the first two rows for seg1_start and seg1_end
# seg1_start = (df.iloc[0, 0], df.iloc[0, 1])
# seg1_end = (df.iloc[1, 0], df.iloc[1, 1])
# print("seg1_start: ",seg1_start, "seg1_end:", seg1_end)

seg2_start = (15.887, -11.585)
seg2_end = (16.009, -11.682)

center_x, center_y = 17.6, -9.5
circle_center = (center_x, center_y)
radius = 5.4 / 2
num_points = 21  # 21 allows for CCC and CC to be unequal odd near the middle
circle_segments = calculate_circle_arc((center_x, center_y ), radius, num_points)
#total_distance = print_segment_coordinates(circle_segments)
#print("total_distance", total_distance)
# Initialize a list to store intersection points
intersection_points = []
intersecting_segments = []
intersecting_index = []
cir_seg_intersects = []

# Loop through the rows of the DataFrame/.csv file and compare each segment with the segments of the circle looking for intersection points
for i in range(len(df) - 1):
    seg1_start = (df.iloc[i, 0], df.iloc[i, 1])
    seg1_end = (df.iloc[i + 1, 0], df.iloc[i + 1, 1])
    cir_seg_ndx = 1
    for this_segment in circle_segments:
        seg2_start, seg2_end = this_segment
        intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
        if intersects_sw:
            #print(f"Intersection found between segments at point {intersects_pt}")
            intersection_points.append(intersects_pt)
            intersecting_segments.append((seg1_start,seg1_end))
            intersecting_index.append(i)
            cir_seg_intersects.append(cir_seg_ndx)
        cir_seg_ndx = cir_seg_ndx + 1

print("intersection_points", intersection_points)
print("the rows in .csv file that intersected with the circle: ", intersecting_index)
print("The circle segments with intersects:", cir_seg_intersects)

'''
use the the circle data, plus the start and end points of the intersecting line segment to calculate the shortest path around the circle
'intersection_points' defines the line segments that intersect with the circle.  Since the line segments are small there is a small line 
segment at the entry point and another line segment at the exit point of the circle.  I need to feed the function 


I need to 
def calculate_circle_arc(circle_center, radius, start_point, end_point, num_points=20):

I need to build the path that goes around the circle at the intersect points
'num_points' = the numbr of segments for the circle (i.e. obstacle)
'first segment' = the index reference for the first intersection point (e.g. 1-21) which is the entrance point to the circle
'second_segment' = Similar to above but is the exit point 
1. Calculate which is shorter
2. Build the list of points around the circle
3. Delete the unneeded points in the path
4. Insert the circle path
'''

intersecting_index_pointer = 0  # I will use this to ID which intersecting_index value is being referenced
# Create an empty DataFrame
revised_path = pd.DataFrame()
revised_path = df.iloc[0:intersecting_index[intersecting_index_pointer]].copy()
column_names = revised_path.columns
print("column_names - revised_path: ",column_names)
additional_records = []
print("type(revised_path): ", type(revised_path))
for i in range(0, len(intersection_points), 2):
    point1 = intersection_points[i]
    point2 = intersection_points[i + 1]
    circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_points)
    #print("circle_arc: ", circle_arc)    
    # so now I have the shortest path I need to append it to 'revised_path'
    circle_arc_df = pd.DataFrame(circle_arc, columns=['x', 'y'])   # Step 1: Convert circle_arc to a DataFrame
    circle_arc_df['angle'] = 0.0  # Step 2: Add a 'z' column with 0.0 as the placeholder
    #print("type(revised_path): ", type(revised_path))
    column_names = circle_arc_df.columns
    print("column_names - circle_arc_df: ",column_names)
    #revised_path = revised_path.append(circle_arc_df, ignore_index=True)  
    # Step 3: Append to revised_path
    revised_path = pd.concat([revised_path, circle_arc_df], ignore_index=True)

    #revised_path = pd.DataFrame.append(revised_path, circle_arc_df, ignore_index=True)

    intersecting_index_pointer += 1
    add_records_start = intersecting_index[intersecting_index_pointer]
    add_records_stop = intersecting_index[intersecting_index_pointer] + 1
    additional_records = df.iloc[add_records_start:add_records_stop].copy()
    #revised_path = revised_path.append(additional_records, ignore_index=True)    
    revised_path = pd.concat([revised_path, additional_records], ignore_index=True)


    plot_path_and_obstacle(circle_center, radius, circle_arc)
    # I need to save the full path
    # I think 'df' is x, y and theta.  I need to calculate theta before appending to the original 
    # where did Site_01_inner_ring_continuous_path_duplicates_removed.csv get its 'theta'
    # look at ~/code_for_testing/archive/path_create_inner_ring_path_v4.py


output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_continuous_obstacles.csv'
with open(output_file_path, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['x', 'y', 'angle'])  # Write header
    for point in revised_path:
        csvwriter.writerow(point)  # Write each point directly  

# Plotting
plt.figure()

# Plot the circle
circle = Circle((center_x, center_y), radius, color='green', fill=False)
plt.gca().add_patch(circle)

# Plot the circle segments
for seg in circle_segments:
    plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'b-')  # Blue line for segments

# Plot the intersection points
for point in intersection_points:
    plt.plot(point[0], point[1], 'ro')  # Red dot for intersection points

# Plot the intersecting segments
for seg in intersecting_segments:
    plt.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]], 'y-')  # Yellow line for intersecting segments    

# # Plot the initial line segment
# plt.plot([seg1_start[0], seg1_end[0]], [seg1_start[1], seg1_end[1]], 'g-')  # Green line for initial segment
plt.title(f'Script: {script_name}\nData source: {csv_file_path}')
plt.legend()
plt.axis('equal')  # Setting equal aspect ratio

plt.show()
