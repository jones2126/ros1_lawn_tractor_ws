# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/plotIntersectingPoints.py
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Define the line segments
line_segments = [
    (-6.373, -3.888, -8.914, 3.491),
    (-5.195, -4.838, -11.075, 12.239),
    (-4.011, -5.805, -10.846, 14.045),
    (-2.86, -6.678, -10.47, 15.425),
    (-1.559, -7.985, -10.556, 18.145),
    (0.136, -10.436, -11.315, 22.819)
]

# Function to create a line given two points
def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

# Function to find the intersection of two lines
def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

# Initialize a list to hold the intersection points
intersections = []

# Plot the segments and calculate intersections
plt.figure(figsize=(10, 8))
for i, (x1, y1, x2, y2) in enumerate(line_segments):
    plt.plot([x1, x2], [y1, y2], marker='o', label=f'Segment {i}')

    # Only calculate intersections if not the last segment
    if i < len(line_segments) - 1:
        next_seg = line_segments[i + 1]
        L1 = line((x2, y2), (x2 + 1, y2))  # Horizontal line from the top point
        L2 = line((next_seg[0], next_seg[1]), (next_seg[2], next_seg[3]))  # Next segment
        intersect = intersection(L1, L2)
        
        # Check if the intersection is within the x-bounds of the next segment
        if intersect and min(next_seg[0], next_seg[2]) <= intersect[0] <= max(next_seg[0], next_seg[2]):
            plt.plot(intersect[0], intersect[1], 'ro')  # plot intersection point
            intersections.append((i, i + 1, *intersect))

# Finalize the plot
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Line Segments and Intersection Points')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

# Create a DataFrame for the intersection points and save to CSV
df_intersections = pd.DataFrame(intersections, columns=['Segment_n', 'Segment_n+1', 'Intersection_X', 'Intersection_Y'])
csv_file_path = 'intersections_corrected.csv'
df_intersections.to_csv(csv_file_path, index=False)

# Print the DataFrame
print(df_intersections)
