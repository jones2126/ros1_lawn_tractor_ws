# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/test_straight_line_check.py
# /home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path_sample_data.csv
import numpy as np
import matplotlib.pyplot as plt

import pandas as pd

# Path to the CSV file
csv_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path_sample_data.csv'
df = pd.read_csv(csv_file_path)  # Read the CSV file
# we need the first two rows for seg1_start and seg1_end
seg1_start = (df.iloc[0, 0], df.iloc[0, 1])
seg1_end = (df.iloc[1, 0], df.iloc[1, 1])
print("seg1_start: ",seg1_start, "seg1_end:", seg1_end)

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
                return True
        return False

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



seg2_start = (15.887, -11.585)
seg2_end = (16.009, -11.682)


# Loop through the rows of the DataFrame and compare each segment with the hardcoded segment
for i in range(len(df) - 1):
    seg1_start = (df.iloc[i, 0], df.iloc[i, 1])
    seg1_end = (df.iloc[i + 1, 0], df.iloc[i + 1, 1])

    # Check for intersection
    intersects_sw, intersects_pt = do_segments_intersect(seg1_start, seg1_end, seg2_start, seg2_end)
    if intersects_sw:
        print(f"Intersection found between segments at point {intersects_pt}")

    # Define the start and end points of the line segments
    seg1_start_ex = seg1_start
    seg1_end_ex = seg1_end
    seg2_start_ex = seg2_start
    seg2_end_ex = seg2_end

    # Plotting the line segments
    plt.figure(figsize=(8, 6))
    plt.plot([seg1_start_ex[0], seg1_end_ex[0]], [seg1_start_ex[1], seg1_end_ex[1]], 'b-', label='Line 1')
    plt.plot([seg2_start_ex[0], seg2_end_ex[0]], [seg2_start_ex[1], seg2_end_ex[1]], 'r-', label='Line 2')

    # Setting plot details
    plt.title("Visualization of Line Segments")
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.axis('equal')  # Setting equal aspect ratio
    plt.legend()
    plt.grid(True)
    plt.xlim(14, 21)
    plt.ylim(-13, -6)

    # Show plot
    plt.show()
