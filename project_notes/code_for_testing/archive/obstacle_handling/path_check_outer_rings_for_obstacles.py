# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/path_check_outer_rings_for_obstacles.py
import pandas as pd
import numpy as np

def is_between(a, b, c):
    # Check if point c is on line segment between a and b
    crossproduct = (c[1] - a[1]) * (b[0] - a[0]) - (c[0] - a[0]) * (b[1] - a[1])
    if abs(crossproduct) > np.finfo(float).eps:
        return False  # c is not on the line
    dotproduct = (c[0] - a[0]) * (b[0] - a[0]) + (c[1] - a[1]) * (b[1] - a[1])
    if dotproduct < 0:
        return False  # c is before a
    squaredlengthba = (b[0] - a[0])**2 + (b[1] - a[1])**2
    if dotproduct > squaredlengthba:
        return False  # c is after b
    return True  # c is between a and b

def find_line_circle_intersections(circle_center, radius, start_point, end_point):
    h, k = circle_center
    r = radius

    # Extracting coordinates from start and end points
    start_x, start_y = start_point
    end_x, end_y = end_point

    # Calculate the coefficients of the line (y = mx + b)
    if end_x != start_x:  # Non-vertical line
        m = (end_y - start_y) / (end_x - start_x)
        b = start_y - m * start_x

        # Quadratic equation coefficients (Ax^2 + Bx + C = 0)
        A = 1 + m**2
        B = -2*h + 2*m*(b - k)
        C = h**2 + (b - k)**2 - r**2

        # Solving the quadratic equation for x
        discriminant = B**2 - 4*A*C
        if discriminant < 0:  # No real intersections
            return [], []
        else:
            x1 = (-B + np.sqrt(discriminant)) / (2*A)
            x2 = (-B - np.sqrt(discriminant)) / (2*A)
            y1 = m*x1 + b
            y2 = m*x2 + b
            intersections = [(x1, y1), (x2, y2)]
    else:  # Vertical line
        x = start_x
        discriminant = r**2 - (x - h)**2
        if discriminant < 0:  # No real intersections
            return [], []
        else:
            y1 = k + np.sqrt(discriminant)
            y2 = k - np.sqrt(discriminant)
            intersections = [(x, y1), (x, y2)]

    # Filtering intersections that are on the segment
    intersections_on_segment = [pt for pt in intersections if is_between(start_point, end_point, pt)]
    return intersections, intersections_on_segment

center_x, center_y = 17.6, -9.5
radius = 5.4 / 2
path_data_file = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path.csv'
path_data = pd.read_csv(path_data_file)

for i in range(len(path_data) - 1):
#for i in range(1292, 1294):
    segment_start = (path_data.iloc[i]['x'], path_data.iloc[i]['y'])
    segment_end = (path_data.iloc[i + 1]['x'], path_data.iloc[i + 1]['y'])
    intersections, intersections_on_segment = find_line_circle_intersections((center_x, center_y), radius, segment_start, segment_end)
    if intersections:
        print(f"segment starting x: {segment_start[0]}, y: {segment_start[1]}, ending x: {segment_end[0]}, y: {segment_end[1]} at index {i}, intersection point {intersections}")
        #print("intersections: ", intersections)
