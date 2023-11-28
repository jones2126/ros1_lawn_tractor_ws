import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

"""
This script calculates the perpendicular intersection points between sequential line segments.
For each line segment except the last one, it finds a line perpendicular to the segment that intersects
with the next segment. It plots these line segments and the intersection points, then saves the intersection
point coordinates to a CSV file.

# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/plotIntersectingPointsV2.py
"""

# Define the line segments
# file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_path_edited.csv'
# line_segments = pd.read_csv(file_path)

line_segments = [
    (-6.373, -3.888, -8.914, 3.491),
    (-5.195, -4.838, -11.075, 12.239),
    (-4.011, -5.805, -10.846, 14.045),
    (-2.86, -6.678, -10.47, 15.425),
    (-1.559, -7.985, -10.556, 18.145),
    (0.136, -10.436, -11.315, 22.819)
]

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
        # Check for the case where the original line is horizontal
        if slope != 0:
            return -1 / slope
        else:
            # The perpendicular line would be vertical
            return None
    else:
        # The original line is vertical, so the perpendicular is horizontal
        return 0

# Find the intersection of two lines given slopes and y-intercepts
def intersection_of_two_lines(slope1, intercept1, slope2, intercept2):
    if slope1 != slope2:  # Ensure lines are not parallel
        # Calculate the intersection point
        x_intersect = (intercept2 - intercept1) / (slope1 - slope2)
        y_intersect = slope1 * x_intersect + intercept1
        return x_intersect, y_intersect
    else:
        return None

# Initialize the list for corrected intersection points
intersections = []

# Plot the segments and calculate the perpendicular intersections
plt.figure(figsize=(10, 8))
for i in range(len(line_segments) - 1):
    # Current segment
    seg1 = line_segments[i]
    # Next segment
    seg2 = line_segments[i + 1]

        # seg_n1 = line_segments.iloc[i + 1]
        # seg_n = line_segments.iloc[i]

    # Calculate the slope and y-intercept of the current segment
    m1, b1 = slope_intercept(seg1[0], seg1[1], seg1[2], seg1[3])
    # Get the perpendicular slope
    m_perp = perpendicular_slope(m1)

    # Calculate the y-intercept of the perpendicular line
    if m_perp is not None:
        b_perp = seg1[3] - m_perp * seg1[2]  # y - mx

        # Calculate the slope and y-intercept of the next segment
        m2, b2 = slope_intercept(seg2[0], seg2[1], seg2[2], seg2[3])

        # Find the intersection of the perpendicular line with the next segment
        if m2 is not None:
            intersect = intersection_of_two_lines(m_perp, b_perp, m2, b2)
            if intersect:
                # Check if the intersection point lies within the x-bounds of the next segment
                if min(seg2[0], seg2[2]) <= intersect[0] <= max(seg2[0], seg2[2]):
                    plt.plot(intersect[0], intersect[1], 'ro')  # plot intersection point
                    intersections.append((i, i + 1, *intersect))

# Plot the original segments
for seg in line_segments:
    plt.plot([seg[0], seg[2]], [seg[1], seg[3]], 'b-o')

# Finalize the plot
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Line Segments and Perpendicular Intersection Points')
plt.grid(True)
plt.axis('equal')
plt.show()

# Create a DataFrame for the corrected intersection points and save to CSV
df_intersections = pd.DataFrame(intersections, columns=['Segment_n', 'Segment_n+1', 'Intersection_X', 'Intersection_Y'])
csv_file_path_corrected = 'intersections_perpendicular_top_n.csv'

df_intersections['section'] = 'top'   # Add 'section' column with all values set to 'top'
df_intersections['ref_line'] = 'n'  # Add 'ref_line' column with all values set to 'n'
csv_file_path = 'intersections_perpendicular_top_n.csv'
df_intersections.to_csv(csv_file_path, index=False)
