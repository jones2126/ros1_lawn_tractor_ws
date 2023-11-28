import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


"""
This script processes a set of line segments defined by their endpoints. It computes the line perpendicular
to each segment (n+1) that passes through its top y-value point. It then calculates the intersection
of this perpendicular line with the previous line segment (n). The intersection points are calculated
only if they lie within the x-bounds of segment n and the y-bounds of segment n+1. The found intersection
points are plotted and saved to a CSV file. If no intersection is found within these bounds, the script
will simply not include an intersection point for that pair of segments.

# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/plotIntersectingPointsV4.py


"""

# Define the line segments data
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_path_edited.csv'
line_segments_data = pd.read_csv(file_path)

# new_line_segments = [
#     (39.902, -2.355, 22.921, 46.96),
#     (40.077, -0.391, 23.989, 46.331),
#     (40.218, 1.67, 25.389, 44.736),
#     (40.604, 3.02, 26.581, 43.746),
#     (41.115, 4.006, 27.559, 43.376),
#     (41.71, 4.752, 28.605, 42.812),
#     (42.288, 5.543, 29.603, 42.384),
#     (42.368, 7.782, 42.017, 8.802)
# ]

# Function to calculate the slope and y-intercept of a line given two points
def slope_intercept(x1, y1, x2, y2):
    if x2 != x1:
        slope = (y2 - y1) / (x2 - x1)
        y_intercept = y1 - slope * x1
        return slope, y_intercept
    else:
        return None, None

# Calculate the perpendicular slope given the original slope
def perpendicular_slope(slope):
    if slope is not None:
        if slope != 0:
            return -1 / slope
        else:
            return None
    else:
        return 0

# Function to find the intersection of two lines given slopes and y-intercepts
#If the slopes of the current segment and the perpendicular line are not the same (i.e., they are not parallel), 
#calculate the intersection point of these two lines using the intersection_of_two_lines function.
def intersection_of_two_lines(slope1, intercept1, slope2, intercept2):
    if slope1 != slope2:  # Ensure lines are not parallel
        x_intersect = (intercept2 - intercept1) / (slope1 - slope2)
        y_intersect = slope1 * x_intersect + intercept1
        return x_intersect, y_intersect
    else:
        return None

# Function to calculate the perpendicular intersections
def find_intersections(line_segments):
    intersections = []
    for i in range(len(line_segments) - 1):
        seg_n1 = line_segments.iloc[i + 1]
        seg_n = line_segments.iloc[i]
        m_n1, b_n1 = slope_intercept(seg_n1['x1'], seg_n1['y1'], seg_n1['x2'], seg_n1['y2'])
        m_perp_n1 = perpendicular_slope(m_n1)
        if m_perp_n1 is not None and m_perp_n1 != 0:
            y_greater_n1 = max(seg_n1['y1'], seg_n1['y2'])
            x_greater_n1 = seg_n1['x1'] if seg_n1['y1'] == y_greater_n1 else seg_n1['x2']

            """
            The statement above checks if the y-coordinate of the first endpoint of the next segment (seg_n1['y1']) is equal to the maximum 
            y-coordinate of that segment (y_greater_n1). The maximum y-coordinate is determined by comparing the y-coordinates 
            of both endpoints of seg_n1.

            If seg_n1['y1'] is equal to y_greater_n1 (meaning the first endpoint has the greater y-value), then x_greater_n1 is 
            assigned the x-coordinate of the first endpoint (seg_n1['x1']).

            If seg_n1['y1'] is not equal to y_greater_n1 (meaning the second endpoint has the greater y-value), then x_greater_n1 
            is assigned the x-coordinate of the second endpoint (seg_n1['x2']).
            """

            b_perp_n1 = y_greater_n1 - m_perp_n1 * x_greater_n1
            m_n, b_n = slope_intercept(seg_n['x1'], seg_n['y1'], seg_n['x2'], seg_n['y2'])
            if m_n is not None:
                intersect = intersection_of_two_lines(m_perp_n1, b_perp_n1, m_n, b_n)
                if intersect:
                    if (min(seg_n['x1'], seg_n['x2']) <= intersect[0] <= max(seg_n['x1'], seg_n['x2'])) and \
                       (min(seg_n1['y1'], seg_n1['y2']) <= intersect[1] <= max(seg_n1['y1'], seg_n1['y2'])):
                        intersections.append((i + 1, i, *intersect))
    return intersections

# Function to plot the line segments and intersections
def plot_segments_and_intersections(line_segments, intersections):
    plt.figure(figsize=(12, 8))
    # Plot line segments
    for i, seg in line_segments.iterrows():
        plt.plot([seg['x1'], seg['x2']], [seg['y1'], seg['y2']], 'b-o')
    # Plot intersection points
    for intersection in intersections:
        plt.plot(intersection[2], intersection[3], 'ro')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Line Segments and Intersection Points')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Calculate the intersections
intersections = find_intersections(line_segments_data)

# Now, plot the line segments and intersections
plot_segments_and_intersections(line_segments_data, intersections)

# Prepare the intersections data for CSV output
df_intersections = pd.DataFrame(intersections, columns=['Segment_n+1', 'Segment_n', 'Intersection_X', 'Intersection_Y'])
df_intersections['section'] = 'top'   # Add 'section' column with all values set to 'top'
df_intersections['ref_line'] = 'n+1'  # Add 'ref_line' column with all values set to 'n+1'
csv_file_path = 'adjusted_intersections_perpendicular.csv'
df_intersections.to_csv(csv_file_path, index=False)
