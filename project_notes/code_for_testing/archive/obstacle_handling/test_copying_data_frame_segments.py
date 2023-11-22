# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/test_copying_data_frame_segments.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import os
script_name = os.path.basename(__file__)
print(f"Start of job; Running script: {script_name}")

def plot_xy_coordinates(csv_file):
    # Load the data from the CSV file
    data = pd.read_csv(csv_file)

    # Plot the x and y coordinates
    plt.figure(figsize=(10, 6))
    plt.scatter(data['x'], data['y'], alpha=0.6)
    plt.title('X and Y Coordinates')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')  # Setting equal aspect ratio
    plt.grid(True)
    plt.show()

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


# File paths
csv_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_inner_ring_continuous_path_duplicates_removed.csv'
output_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_continuous_obstacles.csv'
# List of tuples

intersection_points = [(19.976415271424152, -8.281316268910592), (14.970122812435273, -10.031980848604416), (20.247204353386163, -9.149723771641105), (15.30721630539642, -10.876582335577522), (20.20108302910253, -10.156271221377011), (16.02686940922225, -11.660327881827119)]


circle_arc = [(20.00250335950519, -8.267937660844108), (19.812440265985888, -7.95238633068705), (19.581194638951494, -7.665642400569107), (19.31307090231435, -7.413043344090756), (19.01305993195935, -7.199291059544681), (18.686746155195777, -7.028364348418805), (18.340203601797274, -6.903444853678947), (17.979882841543805, -6.826857836421601), (17.612490912818938, -6.800028893284791), (17.2448664772808, -6.823457420282455), (16.883852524487242, -6.896707317008573), (16.53616899596117, -7.018415104243715), (16.208287699681446, -7.18631530386225), (15.90631184335072, -7.39728260861666), (15.63586242881717, -7.647390056847366), (15.401973622313168, -7.931982129249512), (15.208999048097294, -8.245761407067882), (15.060530749757067, -8.582887178657614), (14.95933232763295, -8.937084158940628), (14.907287496948749, -9.301759298045377), (14.905365024191358, -9.670124504844978), (14.95360069441675, -10.03532300100826)]
intersecting_index = [1291, 1349, 3216, 3291, 4767, 4808]
center_x, center_y = 17.6, -9.5
circle_center = (center_x, center_y)
radius = 5.4 / 2
num_points = 21  # 21 allows for CCC and CC to be unequal odd near the middle

original_data = pd.read_csv(csv_file_path)  				  	# Read the CSV file into a DataFrame
revised_path = pd.DataFrame()  								  	# Initialize an empty DataFrame for revised_path

# Segment 1
temp_path = original_data.iloc[0:intersecting_index[0]].copy()	# Copy rows into a new DataFrame
# need statement to get correct data for 'circle_arc'
i = 0
point1 = intersection_points[i]
point2 = intersection_points[i + 1]
circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_points)
circle_arc_df = pd.DataFrame(circle_arc, columns=['x', 'y'])  	# Convert 'circle_arc' to DataFrame
circle_arc_df['angle'] = 0.0                                  	# Add a 'angle' column with 0.0 as the placeholder
revised_path = pd.concat([revised_path, temp_path, circle_arc_df], ignore_index=True)

# Segment 2
temp_path = original_data.iloc[intersecting_index[1]+1:intersecting_index[2]].copy()				# Copy rows into a new DataFrame
# need statement to get correct data for 'circle_arc'
i = 2
point1 = intersection_points[i]
point2 = intersection_points[i + 1]
circle_arc = calculate_shortest_path(circle_center, radius, point1, point2, num_points)
circle_arc_df = pd.DataFrame(circle_arc, columns=['x', 'y'])	# Convert 'circle_arc' to DataFrame
circle_arc_df['angle'] = 0.0									# Add a 'angle' column with 0.0 as the placeholder
revised_path = pd.concat([revised_path, temp_path, circle_arc_df], ignore_index=True)

# Segment 3
temp_path = original_data.iloc[intersecting_index[3]+1:intersecting_index[4]].copy()              # Copy rows into a new DataFrame
# need statement to get correct data for 'circle_arc'
i = 4
point1 = intersection_points[i]
point2 = intersection_points[i + 1]
circle_arc_df = pd.DataFrame(circle_arc, columns=['x', 'y'])  	# Convert 'circle_arc' to DataFrame
circle_arc_df['angle'] = 0.0                                  	# Add a 'angle' column with 0.0 as the placeholder
revised_path = pd.concat([revised_path, temp_path, circle_arc_df], ignore_index=True)

#Segment 4 
temp_path = original_data.iloc[intersecting_index[5]+1:].copy() # Copy rows into a new DataFrame
revised_path = pd.concat([revised_path, temp_path], ignore_index=True)


revised_path.to_csv(output_file_path, index=False)   			# Save 'revised_path' to the CSV file

plot_xy_coordinates(output_file_path)

Print("eoj")