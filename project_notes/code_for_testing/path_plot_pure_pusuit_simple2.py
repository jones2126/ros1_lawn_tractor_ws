#!/usr/bin/env python3

# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_plot_pure_pusuit_simple2.py

from math import sqrt, atan, sin, pi, cos, acos, atan2
import matplotlib.pyplot as plt
import numpy as np

def plot_circle(center, radius):
    theta = np.linspace(0, 2*np.pi, 100)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    plt.plot(x, y, linestyle='--', label='Circle Trajectory')

# Function to calculate the angle between two vectors
def angle_between(v1, v2):
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    magnitude_v1 = sqrt(v1[0]**2 + v1[1]**2)
    magnitude_v2 = sqrt(v2[0]**2 + v2[1]**2)
    return acos(dot_product / (magnitude_v1 * magnitude_v2))    
P1 = (15.6, 1.3)    # path point 1
P2 = (15.5, 2.3)    # path point 2
#A = (21.0, 6.2)    # current position
A = (14.9, -0.1)     # current position

look_ahead_distance = 4
wheelbase = 1.27
#current_yaw = .9
current_yaw = 1.57

# Calculating the closest point (projecting the current position onto the path)
if P2[0] - P1[0] == 0:  # Check if the path is a vertical line
    closest_point_x = P1[0]
    closest_point_y = A[1]
    closest_point_y = max(min(closest_point_y, P2[1]), P1[1])  # Ensure the closest point lies within the path segment
else:
    m_path = (P2[1] - P1[1]) / (P2[0] - P1[0])      # Slope of the path line
    b_path = P1[1] - m_path * P1[0]                 # y-intercept of the path line
    m_perp = -1 / m_path                            # Slope of the perpendicular line
    b_perp = A[1] - m_perp * A[0]                   # y-intercept of the perpendicular line
    closest_point_x = (b_perp - b_path) / (m_path - m_perp)  # Solving for the intersection (closest point)
    closest_point_y = m_path * closest_point_x + b_path
closest_point = (closest_point_x, closest_point_y)

# Calculating the goal point (B) by moving up the path by the look-ahead distance
dir_vector = (P2[0] - P1[0], P2[1] - P1[1])  # Direction vector of the path
dir_magnitude = sqrt(dir_vector[0]**2 + dir_vector[1]**2)  # Magnitude of the direction vector
unit_dir_vector = (dir_vector[0] / dir_magnitude, dir_vector[1] / dir_magnitude)  # Unit direction vector
# Calculating the goal point (B) by moving along the path by the look-ahead distance
goal_point_x = closest_point_x + unit_dir_vector[0] * look_ahead_distance
goal_point_y = closest_point_y + unit_dir_vector[1] * look_ahead_distance
# Ensure the goal point lies within the path segment
goal_point_x = max(min(goal_point_x, max(P1[0], P2[0])), min(P1[0], P2[0]))
goal_point_y = max(min(goal_point_y, max(P1[1], P2[1])), min(P1[1], P2[1]))
goal_point = (goal_point_x, goal_point_y)

# Calculating the circle center (C)
# Midpoint between A and B
midpoint_x = (A[0] + goal_point_x) / 2
midpoint_y = (A[1] + goal_point_y) / 2
# Slope of the line AB
slope_AB = (goal_point_y - A[1]) / (goal_point_x - A[0]) if goal_point_x != A[0] else float('inf')
# Slope of the perpendicular bisector
slope_perp_bisector = -1 / slope_AB if slope_AB != 0 else float('inf')
# Distance from the midpoint to the circle center
dist_to_center_squared = look_ahead_distance**2 - ((midpoint_x - A[0])**2 + (midpoint_y - A[1])**2) / 4
if dist_to_center_squared < 0:
    print("Error: Cannot find a suitable circle center. Try increasing the look-ahead distance or correcting the initial position.")
    # in the real world I would have to have a more elegant way to handle this
else:
    dist_to_center = sqrt(dist_to_center_squared)

# Determine the direction to move to get the correct turn direction
# If the vehicle is to the right of the path, we want a left turn, so we move in the opposite direction of the slope
if A[0] > P1[0]:
    dist_to_center = -dist_to_center

# Circle center coordinates
circle_center_x = midpoint_x + dist_to_center * sqrt(1 / (1 + slope_perp_bisector**2))
circle_center_y = midpoint_y + slope_perp_bisector * (circle_center_x - midpoint_x)
circle_center = (circle_center_x, circle_center_y)
radius = sqrt((circle_center_x - A[0])**2 + (circle_center_y - A[1])**2)  # Calculate the radius to plot the circle

# Calculate the angle to the goal
# Calculate the vector from the current position (A) to the goal point (B) and robot
vector_to_goal = (goal_point[0] - A[0], goal_point[1] - A[1])
vector_to_robot = (A[0] - closest_point_x, A[1] - closest_point_y)
# Cross product between the direction vector of the path and the vector_to_robot
cross_product = dir_vector[0] * vector_to_robot[1] - dir_vector[1] * vector_to_robot[0]

# Calculate the vector representing the direction of the path
dir_vector_path = (P2[0] - P1[0], P2[1] - P1[1])
# Calculate the angle between these two vectors
angle_to_goal = angle_between(vector_to_goal, dir_vector_path)
# Determine the sign of the angle based on the relative position of the robot to the path
# If the robot is to the right of the path, we want a left turn, so we negate the angle

# Calculate the off-track error
off_track_error = sqrt((closest_point[0] - A[0])**2 + (closest_point[1] - A[1])**2)

#if A[0] > closest_point_x:
if cross_product > 0:  # Robot is to the right of the path
    angle_to_goal = -angle_to_goal
    off_track_error = -off_track_error

# Calculate the steering angle using the pure pursuit formula
steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)

print("Target Steering Angle: {:.3f}".format(steering_angle))
print("Current Position(A):", A)
print("Goal Position(B):", goal_point[0], goal_point[1])
print("Off_Track_Error: {:.2f}".format(off_track_error))

# Plotting the vehicle's new target steering path
# Normalize the direction vector of the path
dir_magnitude = sqrt(dir_vector[0]**2 + dir_vector[1]**2)
unit_dir_vector = (dir_vector[0] / dir_magnitude, dir_vector[1] / dir_magnitude)

# Compute the steering direction vector by rotating the normalized path's direction vector by the steering angle
steering_direction_vector = (
    unit_dir_vector[0] * cos(steering_angle) - unit_dir_vector[1] * sin(steering_angle),
    unit_dir_vector[0] * sin(steering_angle) + unit_dir_vector[1] * cos(steering_angle)
)

# Scale the steering direction vector by the desired length
length = 4  # Length of the line representing the vehicle's steering direction; adjust as needed
steering_direction_vector_scaled = (steering_direction_vector[0] * length, steering_direction_vector[1] * length)

# Plot the line from the current position (A) to the end of the steering direction vector
steering_line_end = (A[0] + steering_direction_vector_scaled[0], A[1] + steering_direction_vector_scaled[1])
plt.plot([A[0], steering_line_end[0]], [A[1], steering_line_end[1]], 'y--', linewidth=2, label="Target steering path")

plt.plot([P1[0], P2[0]], [P1[1], P2[1]], 'b-', label='Path')  # Plotting the path
plt.plot(A[0], A[1], 'ro', label='Current Position (A)')  # Plotting the current position (A)
plt.plot(closest_point[0], closest_point[1], 'go', label='Closest Point')  # Plotting the closest point
plt.plot(goal_point[0], goal_point[1], 'mo', label='Goal Point (B)')  # Plotting the goal point (B)
plot_circle(circle_center, radius)  # Plotting the circle that passes through A and B
plt.plot(circle_center[0], circle_center[1], 'co', label='Circle Center (C)')  # Plotting the circle's center (midpoint)

# Plotting the line representing the vehicle's heading
length = 4  # Length of the line representing the vehicle's heading
heading_line_end = (A[0] + length * cos(current_yaw), A[1] + length * sin(current_yaw))
plt.plot([A[0], heading_line_end[0]], [A[1], heading_line_end[1]], 'y-', linewidth=2, label="Current Heading")

# Print the current yaw at the end of the line segment
plt.text(heading_line_end[0], heading_line_end[1], f"Current Yaw: {current_yaw:.2f} rad", fontsize=9, color='black')

# Calculate the position for the text on the plot
# Define an offset to move the text away from the line
# I want the text position at the end of the steering direction line 
text_offset_x = -4.0
text_offset_y = 0.0
text_position_x = steering_line_end[0] + text_offset_x
text_position_y = steering_line_end[1] + text_offset_y

# Add the text to the plot
plt.text(text_position_x, text_position_y, f"Steer Angle: {steering_angle:.2f} rad", fontsize=9, color='black')

# Adding labels, legend, and showing the plot
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.title('Pure Pursuit Path Tracking')
plt.grid(True)
plt.axis('equal')

# Calculate the midpoint of the line segment between the closest point and A
midpoint_error_x = (closest_point[0] + A[0]) / 2
midpoint_error_y = (closest_point[1] + A[1]) / 2

# Annotate the off-track error value on the plot with an arrow
plt.annotate(f"Off-track Error: {off_track_error:.2f} m", 
             xy=(midpoint_error_x, midpoint_error_y), 
             xytext=(midpoint_error_x + 1, midpoint_error_y - 2),  # Adjust the y-coordinate value here
             arrowprops=dict(facecolor='red', arrowstyle="->"),
             fontsize=9, color='black')

# Zooming in on the circle by setting x and y axis limits
zoom_percentage = 90  # Define a zoom percentage (e.g., 10%)
# Calculate the bounding box of the circle
x_min = circle_center_x - radius
x_max = circle_center_x + radius
y_min = circle_center_y - radius
y_max = circle_center_y + radius
# Apply the zoom percentage to expand the bounding box
zoom_factor = 1 + zoom_percentage / 100
x_range = (x_max - x_min) * zoom_factor
y_range = (y_max - y_min) * zoom_factor

# Calculate the plot limits based on the midpoint and the calculated range
x_min_zoomed = midpoint_x - x_range / 2
x_max_zoomed = midpoint_x + x_range / 2
y_min_zoomed = midpoint_y - y_range / 2
y_max_zoomed = midpoint_y + y_range / 2

# Set the plot limits
plt.xlim(x_min_zoomed, x_max_zoomed)
plt.ylim(y_min_zoomed, y_max_zoomed)

plt.show()
