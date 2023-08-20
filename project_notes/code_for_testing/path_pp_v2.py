#!/usr/bin/env python3
'''
This program was written to help me decompose the steps needed to calculate
a steering angle for my robot.  The key approach is using the 'pure pursuit' process
that culminates in the statement:

steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)

My robot is an Ackermann steering style robot.  In field testing the steering angle
will be positve for a left turn and negative for a right turn.

I took an approach that only the desired path and current position are used as input
along with constants for wheelbase and look ahead distance.

I then used the result to plot key data elements to somewhat verify the approach.

Al Jones

'''

# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_plot_pure_pusuit_simple.py


import matplotlib.pyplot as plt
from math import sqrt, acos, atan, sin, cos
import numpy as np

# Function to calculate the angle between two 2D points (aka vectors)
def angle_between(v1, v2):
    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    magnitude_v1 = sqrt(v1[0]**2 + v1[1]**2)
    magnitude_v2 = sqrt(v2[0]**2 + v2[1]**2)
    return acos(dot_product / (magnitude_v1 * magnitude_v2))  

def calculate_steer_angle():
    global P1, P2, A  # inputs
    global closest_point, goal_point, circle_center, radius, angle_to_goal, \
    		off_track_error, steering_angle   # outputs
    # Calculating the closest point (projecting the current position onto the path)

    if P2[0] - P1[0] == 0:  # Check if the path is a vertical line
        closest_point_x = P1[0]
        closest_point_y = A[1]
        closest_point_y = max(min(closest_point_y, P2[1]), P1[1])  # Ensure the closest point lies within the path segment
    elif P2[1] - P1[1] == 0:  # Check if the path is a horizontal line
        closest_point_x = A[0]
        closest_point_y = P1[1]
        closest_point_x = max(min(closest_point_x, P2[0]), P1[0])  # Ensure the closest point lies within the path segment
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

    # Calculating the circle center (C) that intesects (A) & (B)
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

    # Calculate the off-track error
    off_track_error = sqrt((closest_point[0] - A[0])**2 + (closest_point[1] - A[1])**2)

    # Determine the sign of the angle based on the relative position of the robot to the path
    # If the robot is to the right of the path, we want a left turn, so we negate the angle
    if cross_product > 0:  # Robot is to the right of the path
        angle_to_goal = -angle_to_goal
        off_track_error = -off_track_error  # Not sure if this is a convention or not

    # Calculate the steering angle using the pure pursuit formula
    steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)


def plot_values():
    global P1, P2, A, closest_point, goal_point, circle_center, radius, angle_to_goal, \
    		off_track_error, steering_angle
    
    # Calculate the axis limits dynamically based on the waypoints
    x_min = min(P1[0], P2[0], A[0], closest_point[0]) - 2
    x_max = max(P1[0], P2[0], A[0], closest_point[0]) + 2
    y_min = min(P1[1], P2[1], A[1], closest_point[1]) - 2
    y_max = max(P1[1], P2[1], A[1], closest_point[1]) + 2
    
    x_min = round(x_min)
    x_max = round(x_max)
    y_min = round(y_min)
    y_max = round(y_max)
    
    # Plot the line segment
    plt.arrow(P1[0], P1[1], P2[0] - P1[0], P2[1] - P1[1], head_width=0.1, head_length=0.2, fc='blue', ec='blue', label='Path')

    # Plot the current position 'A'
    plt.scatter(A[0], A[1], color='red', label='Current Position (A)')
    
    # Plot the closest point
    plt.scatter(closest_point[0], closest_point[1], color='green', label='Closest Point')

    # Plot the goal point
    plt.scatter(goal_point[0], goal_point[1], color='purple', label='Goal Point (B)')

    # Adding a text box with the angle_to_goal and off_track_error values
    text_str = f"Angle to Goal: {angle_to_goal:.2f} rad\nOff-track Error: {off_track_error:.2f} meters"
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    plt.text(x_min + 0.1, y_min + 0.1, text_str, fontsize=9, verticalalignment='bottom', bbox=props)
    

    # Plot the circle center
    plt.scatter(circle_center[0], circle_center[1], color='blue', label='Circle Center (C)')
    theta = np.linspace(0, 2*np.pi, 100)
    x = circle_center[0] + radius * np.cos(theta)
    y = circle_center[1] + radius * np.sin(theta)
    plt.plot(x, y, linestyle='--', label='Circle Trajectory')   


    # plot a simple line segment from the current position following the new steer angle
    # Normalize the direction vector of the path
    dir_vector = (P2[0] - P1[0], P2[1] - P1[1])
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
    steering_line_end = (A[0] + steering_direction_vector_scaled[0], A[1] + steering_direction_vector_scaled[1])
    plt.plot([A[0], steering_line_end[0]], [A[1], steering_line_end[1]], 'y--', linewidth=2, label="Target steering path")

    # Setting title, legend, grid, and axis properties
    plt.title("Pure Pursuit Path Tracking")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.xticks(range(x_min, x_max+1))
    plt.yticks(range(y_min, y_max+1))
    
    # Displaying the plot
    plt.show()

# Given mission data
mission = [
    ((0.1, -0.1), (-0.1,  5.1),   (.2,   1.9)),
    ((0.1, -0.1), (-5.1,  0.1), (-1.9,   0.2)),
    ((0.1, -0.1), (-0.1, -5.1),   (.2,  -1.9))
]
'''
# more elaborate test data
mission = [((0, 0), (5.0, 0.0), (0.0, 1.0)),
 ((0, 0), (5.0, 0.0), (0.0, -1.0)),
 ((0, 0),
  (4.095760221444959, 2.8678821817552302),
  (-0.573576436351046, 0.8191520442889918)),
 ((0, 0),
  (4.095760221444959, 2.8678821817552302),
  (0.573576436351046, -0.8191520442889918)),
 ((0, 0),
  (1.7101007166283442, 4.698463103929542),
  (-0.9396926207859083, 0.3420201433256688)),
 ((0, 0),
  (1.7101007166283442, 4.698463103929542),
  (0.9396926207859083, -0.3420201433256688)),
 ((0, 0),
  (-1.2940952255126033, 4.8296291314453415),
  (-0.9659258262890683, -0.25881904510252063)),
 ((0, 0),
  (-1.2940952255126033, 4.8296291314453415),
  (0.9659258262890683, 0.25881904510252063)),
 ((0, 0),
  (-3.8302222155948895, 3.2139380484326976),
  (-0.6427876096865395, -0.7660444431189779)),
 ((0, 0),
  (-3.8302222155948895, 3.2139380484326976),
  (0.6427876096865395, 0.7660444431189779)),
 ((0, 0),
  (-4.9809734904587275, 0.43577871373829097),
  (-0.0871557427476582, -0.9961946980917455)),
 ((0, 0),
  (-4.9809734904587275, 0.43577871373829097),
  (0.0871557427476582, 0.9961946980917455)),
 ((0, 0), (-5.0, -6.123233995736766e-16), (1.2246467991473532e-16, -1.0)),
 ((0, 0), (-5.0, -6.123233995736766e-16), (-1.2246467991473532e-16, 1.0)),
 ((0, 0),
  (-4.095760221444958, -2.867882181755232),
  (0.5735764363510464, -0.8191520442889916)),
 ((0, 0),
  (-4.095760221444958, -2.867882181755232),
  (-0.5735764363510464, 0.8191520442889916)),
 ((0, 0),
  (-1.7101007166283413, -4.698463103929543),
  (0.9396926207859085, -0.34202014332566827)),
 ((0, 0),
  (-1.7101007166283413, -4.698463103929543),
  (-0.9396926207859085, 0.34202014332566827)),
 ((0, 0),
  (1.294095225512607, -4.829629131445341),
  (0.9659258262890681, 0.2588190451025214)),
 ((0, 0),
  (1.294095225512607, -4.829629131445341),
  (-0.9659258262890681, -0.2588190451025214)),
 ((0, 0),
  (3.830222215594893, -3.2139380484326936),
  (0.6427876096865387, 0.7660444431189786)),
 ((0, 0),
  (3.830222215594893, -3.2139380484326936),
  (-0.6427876096865387, -0.7660444431189786)),
 ((0, 0),
  (4.980973490458728, -0.4357787137382859),
  (0.08715574274765718, 0.9961946980917457)),
 ((0, 0),
  (4.980973490458728, -0.4357787137382859),
  (-0.08715574274765718, -0.9961946980917457))]
'''  
# constants
look_ahead_distance = 2
wheelbase = 1.27

# Plotting each line segment one by one with dynamically calculated axis limits
for waypoints in mission:
	P1, P2, A = waypoints
	calculate_steer_angle()
	plot_values()
