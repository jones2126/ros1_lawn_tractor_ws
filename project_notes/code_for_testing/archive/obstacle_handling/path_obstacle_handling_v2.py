
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def line_circle_intersection(line_start, line_end, circle_center, circle_radius):
    # Function to calculate line-circle intersections
    x1, y1 = line_start
    x2, y2 = line_end
    cx, cy = circle_center

    A = y2 - y1
    B = x1 - x2
    C = x2*y1 - x1*y2

    a = A*A + B*B
    b = 2 * (A*C + A*B*cy - B*B*cx)
    c = C*C + 2*B*C*cy - B*B*(circle_radius**2 + cx**2 + cy**2)

    discriminant = b*b - 4*a*c
    if discriminant < 0:
        return []

    t1 = (-b + np.sqrt(discriminant)) / (2*a)
    t2 = (-b - np.sqrt(discriminant)) / (2*a)

    intersections = []
    for t in [t1, t2]:
        x = t
        y = (-C - A*t) / B
        if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
            intersections.append((x, y))

    return intersections

# Load your data
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_path_with_intercepts2.csv'
line_segments_data = pd.read_csv(file_path)

# Obstacle parameters (smaller obstacle)
obstacle_center = (19.7, -5.5)
obstacle_radius = 0.5 / 2  # Diameter is 0.5 meters, radius is half


# Visualization of line segments and obstacle before processing
fig, ax = plt.subplots(figsize=(12, 8))
for index, row in line_segments_data.iterrows():
    plt.plot([row['x1'], row['x2']], [row['y1'], row['y2']], 'b-', alpha=0.5)
obstacle_plot = patches.Circle(obstacle_center, obstacle_radius, color='red', alpha=0.3)
ax.add_patch(obstacle_plot)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Line Segments and Obstacle')
plt.grid(True)
plt.axis('equal')
plt.show()



# Process each line segment and check for intersections
updated_segments = []
for index, row in line_segments_data.iterrows():
    segment_start = (row['x1'], row['y1'])
    segment_end = (row['x2'], row['y2'])
    intersections = line_circle_intersection(segment_start, segment_end, obstacle_center, obstacle_radius)

    # if not intersections:
    #     updated_segments.append(row)
    # else:
    #     if len(intersections) == 2:
    #         inter1, inter2 = intersections
    #         updated_segments.append([row['x1'], row['y1'], inter1[0], inter1[1]] + [np.nan] * 4)
    #         updated_segments.append([inter2[0], inter2[1], row['x2'], row['y2']] + [np.nan] * 4)

    # Print to check if intersections are detected
    print(f"Segment {index}: Start {segment_start}, End {segment_end}, Intersections: {intersections}")

    if not intersections:
        updated_segments.append(row)
    else:
        # Print to confirm entering this block and to see the updated segments
        print(f"Segment {index} intersects. Updating segment.")
        
        if len(intersections) == 2:
            inter1, inter2 = intersections
            updated_segment_1 = [row['x1'], row['y1'], inter1[0], inter1[1]] + [np.nan] * 4
            updated_segment_2 = [inter2[0], inter2[1], row['x2'], row['y2']] + [np.nan] * 4
            updated_segments.append(updated_segment_1)
            updated_segments.append(updated_segment_2)
            print(f"Updated segments: {updated_segment_1}, {updated_segment_2}")

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
