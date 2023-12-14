# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/test_calc_intersecting_pts_v2.py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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

# Read the CSV file into a DataFrame
#line_segments_df = pd.read_csv('path_to_your_csv_file.csv')  # Replace with your CSV file path
line_segments_df = pd.read_csv('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophedon_line_segments_trimmed.csv')

# Distance to the left or right side
distance = 0.45

# Initialize a list to store all new points
all_new_points_list = []
# Add index and point number tracking
for index, row in line_segments_df.iterrows():
    # Extract the end points of the line segment
    x1, y1, x2, y2 = row['x1'], row['y1'], row['x2'], row['y2']
    # Calculate the direction of the line segment
    dx, dy = x2 - x1, y2 - y1
    # Calculate the perpendicular points for P1 and P4
    p4, p1 = calculate_perpendicular_points(x1, y1, dx, dy, distance)
    # Calculate the perpendicular points for P3 and P2
    p3, p2 = calculate_perpendicular_points(x2, y2, dx, dy, distance)

    # Append index, point number and points to the list
    points = [p1, p2, p3, p4]
    for point_number, point in enumerate(points, start=1):
        all_new_points_list.append([index, point_number] + list(point))

# Convert the list of points with index and point number to a DataFrame
all_new_points = pd.DataFrame(all_new_points_list, columns=['index', 'point', 'x', 'y'])

# Save the all new points to a CSV file
csv_output_file = 'all_perpendicular_points_with_index_and_point.csv'
all_new_points.to_csv(csv_output_file, index=False)


# all_new_points_list = []
# # Add index tracking
# for index, row in line_segments_df.iterrows():

#     # Append index and points to the list
#     for point in [p1, p2, p3, p4]:
#         all_new_points_list.append([index] + list(point))

# # Convert the list of points with index to a DataFrame
# all_new_points = pd.DataFrame(all_new_points_list, columns=['index', 'x', 'y'])

# # Save the all new points to a CSV file
# all_new_points.to_csv('all_perpendicular_points_with_index.csv', index=False)



# Optionally, you can plot the line segments and the new points
plt.figure(figsize=(8, 6))
# for index, row in line_segments_df.iterrows():
#     plt.plot([row['x1'], row['x2']], [row['y1'], row['y2']], 'bo-')

for index, row in line_segments_df.iterrows():
    # Existing line plotting code
    plt.plot([row['x1'], row['x2']], [row['y1'], row['y2']], 'bo-') 

    # New code to plot sequential numbers at the endpoints
    plt.text(row['x1'], row['y1'], str(index*2), color='red')   # Number at the first endpoint
    plt.text(row['x2'], row['y2'], str(index*2+1), color='red') # Number at the second endpoint


plt.scatter(all_new_points['x'], all_new_points['y'], color='red')

# New code to plot the continuous green line segment based on selecting key points from .csv file that will be used for Dubins path
df = pd.read_csv(csv_output_file)
x_coords = []
y_coords = []
for i in range(0, len(df), 8):
    if i + 7 >= len(df):
        break
    # Corrected line to capture the right points
    selected_points = df.iloc[[i+1, i+2, i+4, i+7], :]
    x_coords.extend(selected_points['x'].tolist())
    y_coords.extend(selected_points['y'].tolist())
plt.plot(x_coords, y_coords, color='green')


plt.grid(True)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Line Segments with Perpendicular Points')
plt.axis('equal')
plt.show()
