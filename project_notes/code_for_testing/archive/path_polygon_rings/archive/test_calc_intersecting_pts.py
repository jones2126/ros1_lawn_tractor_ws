# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/test_calc_intersecting_pts.py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Function to calculate the slope of a line given two points
def calculate_slope(x1, y1, x2, y2):
    if x2 - x1 == 0:
        return float('inf')  # Infinite slope for vertical lines
    else:
        return (y2 - y1) / (x2 - x1)

# Function to calculate the perpendicular points from the midpoint
def calculate_perpendicular_points(mid_x, mid_y, dx, dy, distance):
    # Determine the length of the line segment
    length = np.sqrt(dx**2 + dy**2)
    # Normalize direction to get the unit vector
    ux, uy = dx / length, dy / length
    # Rotate the unit vector by 90 degrees to get the perpendicular direction
    perp_x, perp_y = -uy, ux
    # Calculate points to the left and right of the midpoint
    left_x, left_y = mid_x - perp_x * distance, mid_y - perp_y * distance
    right_x, right_y = mid_x + perp_x * distance, mid_y + perp_y * distance
    return (left_x, left_y), (right_x, right_y)

# Original points defining the line segment
x1, y1, x2, y2 = -10.1711753055234, -5.78744803434574, -12.8279113117399, 1.92827357402135

# Direction of the original line
dx, dy = x2 - x1, y2 - y1

# Distance to the left or right side
distance = 0.45

# Calculate the perpendicular points for P4 (right of x1, y1) and P1 (left of x1, y1)
p4, p1 = calculate_perpendicular_points(x1, y1, dx, dy, distance)

# Calculate the perpendicular points for P3 (left of x2, y2) and P2 (right of x2, y2)
p3, p2 = calculate_perpendicular_points(x2, y2, dx, dy, distance)

# Create a DataFrame to store the new points in the order P1 through P4
points_df = pd.DataFrame([p1, p2, p3, p4], columns=['x', 'y'])
csv_file_path = 'new_perpendicular_points_ordered.csv'
points_df.to_csv(csv_file_path, index=False)

# Plot the original line segment and the new perpendicular points
plt.figure(figsize=(8, 6))
plt.plot([x1, x2], [y1, y2], 'bo-', label='Original Line Segment')
plt.scatter(points_df['x'], points_df['y'], color='red', label='New Perpendicular Points')

# Annotate the perpendicular points
for i, point in points_df.iterrows():
    plt.annotate(f'P{i+1}', (point['x'], point['y']), textcoords="offset points", xytext=(0,10), ha='center')

plt.grid(True)
plt.legend()
plt.title('Original Line Segment with Perpendicular Points Labeled P1-P4')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.axis('equal')
plt.show()

# Print the path to the CSV file
print(f"The points have been saved to: {csv_file_path}")
