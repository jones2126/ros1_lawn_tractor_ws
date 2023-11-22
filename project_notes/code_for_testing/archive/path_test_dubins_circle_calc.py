# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_test_dubins_circle_calc.py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def calculate_dubins_points(segment1, segment2, turning_radius):
    # Unpack line segment coordinates
    x1, y1, x2, y2 = segment1
    x3, y3, x4, y4 = segment2

    # Calculate direction vectors for each segment
    dir_vec1 = np.array([x2 - x1, y2 - y1])
    dir_vec2 = np.array([x4 - x3, y4 - y3])

    # Normalize direction vectors
    dir_vec1 = dir_vec1 / np.linalg.norm(dir_vec1)
    dir_vec2 = dir_vec2 / np.linalg.norm(dir_vec2)

    # Calculate the normal vectors for each segment
    norm_vec1 = np.array([-dir_vec1[1], dir_vec1[0]])
    norm_vec2 = np.array([-dir_vec2[1], dir_vec2[0]])

    # Calculate potential circle centers for each segment
    circle_center1 = np.array([x1, y1]) + norm_vec1 * turning_radius
    circle_center2 = np.array([x3, y3]) + norm_vec2 * turning_radius

    # Choose the circle center that is on the correct side for transitioning
    circle_center = (circle_center1 + circle_center2) / 2

    # Calculate tangent points to the circle for each segment
    t1 = (np.dot(circle_center - np.array([x1, y1]), dir_vec1) / np.linalg.norm(dir_vec1))
    intersect1 = np.array([x1, y1]) + t1 * dir_vec1

    t2 = (np.dot(circle_center - np.array([x3, y3]), dir_vec2) / np.linalg.norm(dir_vec2))
    intersect2 = np.array([x3, y3]) + t2 * dir_vec2

    return {
        'circle_center': circle_center,
        'intersect1': intersect1,
        'intersect2': intersect2
    }

# Example usage with dummy data
# Replace this with reading your actual CSV file
field_segments = pd.DataFrame({
    'x1': [-6.373, -5.195],
    'y1': [-3.888, -4.838],
    'x2': [-8.914, -11.075],
    'y2': [3.491, 12.239]
})

# Calculate Dubins path components for the first two segments
turning_radius = 1.5  # Turning radius of the vehicle
segment1 = tuple(field_segments.iloc[0])
segment2 = tuple(field_segments.iloc[1])
dubins_points = calculate_dubins_points(segment1, segment2, turning_radius)

# Plot the first few points to show a continuous path
plt.figure(figsize=(12, 8))
plt.plot([segment1[0], segment1[2]], [segment1[1], segment1[3]], 'ro-')  # First line segment
plt.plot([segment2[0], segment2[2]], [segment2[1], segment2[3]], 'ro-')  # Second line segment
plt.plot(dubins_points['circle_center'][0], dubins_points['circle_center'][1], 'bo')  # Circle center
plt.plot(dubins_points['intersect1'][0], dubins_points['intersect1'][1], 'go')  # Tangent point on first segment
plt.plot(dubins_points['intersect2'][0], dubins_points['intersect2'][1], 'go')  # Tangent point on second segment
circle = plt.Circle(dubins_points['circle_center'], turning_radius, fill=False, color='blue', linestyle='--')
plt.gca().add_artist(circle)
plt.axis('equal')
plt.grid(True)
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Continuous Path with Dubins Curves')
plt.show()
