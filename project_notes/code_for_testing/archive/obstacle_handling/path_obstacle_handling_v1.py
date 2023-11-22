# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/path_obstacle_handling_v1.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# def line_circle_intersection(line_start, line_end, circle_center, circle_radius):
#     # Function to calculate line-circle intersections
#     x1, y1 = line_start
#     x2, y2 = line_end
#     cx, cy = circle_center

#     A = y2 - y1
#     B = x1 - x2
#     C = x2*y1 - x1*y2

#     a = A*A + B*B
#     b = 2 * (A*C + A*B*cy - B*B*cx)
#     c = C*C + 2*B*C*cy - B*B*(circle_radius**2 + cx**2 + cy**2)

#     discriminant = b*b - 4*a*c
#     if discriminant < 0:
#         return []

#     t1 = (-b + np.sqrt(discriminant)) / (2*a)
#     t2 = (-b - np.sqrt(discriminant)) / (2*a)

#     intersections = []
#     for t in [t1, t2]:
#         x = t
#         y = (-C - A*t) / B
#         if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
#             intersections.append((x, y))

#     return intersections

def line_circle_intersection(line_start, line_end, circle_center, circle_radius):
    # Convert inputs to numpy arrays for easier calculations
    A = np.array(line_start)
    B = np.array(line_end)
    C = np.array(circle_center)
    R = circle_radius

    # Vector AB
    AB = B - A

    # Project vector AC onto AB to find the closest point D on AB to the circle center C
    AC = C - A
    t = np.dot(AC, AB) / np.dot(AB, AB)
    D = A + t * AB

    # Calculate the distance from D to C
    DC = np.linalg.norm(D - C)

    # If the distance from D to C is greater than the radius, there's no intersection
    if DC > R:
        return []

    # Find the distance from D to the intersection points along AB
    dt = np.sqrt(R**2 - DC**2) / np.linalg.norm(AB)

    # Calculate the intersection points
    intersection1 = D - dt * AB
    intersection2 = D + dt * AB

    # Check if the intersections are within the line segment
    intersections = []
    for P in [intersection1, intersection2]:
        t = np.dot(P - A, AB) / np.dot(AB, AB)
        if 0 <= t <= 1:
            intersections.append(tuple(P))

    return intersections

# Load your data
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_path_with_intercepts2.csv'
line_segments_data = pd.read_csv(file_path)

# Obstacle parameters
obstacle_center = (20, -5)
obstacle_radius = 4 / 2  # Diameter is 4 meters, radius is half

# Process each line segment and check for intersections
updated_segments = []
for index, row in line_segments_data.iterrows():
    segment_start = (row['x1'], row['y1'])
    segment_end = (row['x2'], row['y2'])
    intersections = line_circle_intersection(segment_start, segment_end, obstacle_center, obstacle_radius)

    if not intersections:
        updated_segments.append(row)
    else:
        if len(intersections) == 1:
            intersection = intersections[0]
            if np.linalg.norm(np.array(segment_start) - np.array(obstacle_center)) < obstacle_radius:
                updated_segments.append([intersection[0], intersection[1], row['x2'], row['y2']] + [np.nan] * 4)
            else:
                updated_segments.append([row['x1'], row['y1'], intersection[0], intersection[1]] + [np.nan] * 4)
        else:
            inter1, inter2 = intersections
            updated_segments.append([row['x1'], row['y1'], inter1[0], inter1[1]] + [np.nan] * 4)
            updated_segments.append([inter2[0], inter2[1], row['x2'], row['y2']] + [np.nan] * 4)

# Convert the updated segments into a DataFrame
updated_segments_df = pd.DataFrame(updated_segments, columns=line_segments_data.columns)

# Visualization
fig, ax = plt.subplots(figsize=(12, 8))

for index, row in line_segments_data.iterrows():
    plt.plot([row['x1'], row['x2']], [row['y1'], row['y2']], 'b-', alpha=0.3, label='Original Path' if index == 0 else "")

for index, row in updated_segments_df.iterrows():
    plt.plot([row['x1'], row['x2']], [row['y1'], row['y2']], 'r-', label='Updated Path' if index == 0 else "")

obstacle = patches.Circle(obstacle_center, obstacle_radius, color='green', alpha=0.3, label='Obstacle')
ax.add_patch(obstacle)

plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Original vs Updated Path with Obstacle')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
