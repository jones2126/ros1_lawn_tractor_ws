#!/usr/bin/env python
'''
Script that helps build a path to follow using four corner inputs

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_third_planning_helper.py

'''

import matplotlib.pyplot as plt
from shapely.geometry import Polygon
import math

def calculate_angle(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    angle = math.atan2(dy, dx)
    return angle

def adjust_angle(angle):
    return angle if angle >= 0 else (2 * math.pi + angle)

# Define the coordinates of the lower left and lower right corner points


# Upper left: 12.9, 16.3
# Upper right: 23.3, 18.9
# Lower left: 15.9, 0
# Lower right: 22.3, 0.0
upper_left_x = 12.9
upper_right_x = 23.3
lower_left_x = 15.9
lower_right_x = 22.3

upper_left_y = 16.3
upper_right_y = 18.9
lower_left_y = 0.0
lower_right_y = -0.4

# Calculate the middle x-coordinate
middle_x_lower = (lower_left_x + lower_right_x) / 2
middle_x_upper = (upper_left_x + upper_right_x) / 2
middle_y_lower = (lower_left_y + lower_right_y) / 2
middle_y_upper = (upper_left_y + upper_right_y) / 2

# Print the result
print("The middle coordinate on the lower part of the polygon is:", middle_x_lower, middle_y_lower)
print("The middle coordinate on the upper part of the polygon is:", middle_x_upper, middle_y_upper)
from shapely.geometry import Polygon

# Angle between lower_left and upper_left
angle1 = calculate_angle(lower_left_x, lower_left_y, upper_left_x, upper_left_y)

# Angle between upper_left and upper_right
angle2 = calculate_angle(upper_left_x, upper_left_y, upper_right_x, upper_right_y)

# Angle between upper_right and lower_right
angle3 = calculate_angle(upper_right_x, upper_right_y, lower_right_x, lower_right_y)

# Angle between lower_right and lower_left
angle4 = calculate_angle(lower_right_x, lower_right_y, lower_left_x, lower_left_y)

print("Angle's travelling in a clockwise rotation:")
print("Angle between lower_left and upper_left in radians is:", angle1)
print("Angle between upper_left and upper_right in radians is:", angle2)
print("Angle between upper_right and lower_right in radians is:", angle3)
print("Angle between lower_right and lower_left in radians is:", angle4)


# Define the points starting from lower right and moving counter clockwise
points = [
    (lower_right_x, lower_right_y),
    (upper_right_x, upper_right_y),
    (upper_left_x, upper_left_y),
    (lower_left_x, lower_left_y)
]

# Initialize a list to store the angles
angles = []

# Calculate the angles between adjacent vertices
for i in range(len(points)):
    x1, y1 = points[i]
    x2, y2 = points[(i + 1) % len(points)]  # Next point in counter-clockwise direction
    angle = calculate_angle(x1, y1, x2, y2)
    angles.append(angle)

# Convert angles to degrees if needed
angles_degrees = [math.degrees(angle) for angle in angles]

# Print the calculated angles
for i, angle in enumerate(angles):
    print(f"Angle {i + 1} in radians: {angle}, in degrees: {angles_degrees[i]}")

# Print the calculated angles
for i, angle in enumerate(angles):
    adjusted_angle = adjust_angle(angle)
    print(f"Angle {i + 1} in radians: {adjusted_angle}, in degrees: {math.degrees(adjusted_angle)}")

# Create a polygon
polygon = Polygon([
    (lower_left_x, lower_left_y), 
    (upper_left_x, upper_left_y), 
    (upper_right_x, upper_right_y), 
    (lower_right_x, lower_right_y)
])

# The distance for the inner rings
inset = -1.0  # negative for inside buffering

# Loop to create inner polygons and print their coordinates
ring_count = 4  # adjust as needed to create the number of rings you want

for i in range(1, ring_count + 1):
    inner_polygon = polygon.buffer(i * inset)
    
    # Check if inner polygon is valid (it might not be if the inset is too large)
    if inner_polygon.is_empty:
        print(f"Cannot create ring {i}, polygon is too small.")
        break

    # Extracting and printing the coordinates of the corners of the inner polygon
    x, y = inner_polygon.exterior.xy
    print(f"Ring {i} Coordinates:")
    for xi, yi in zip(x, y):
        print(xi, yi)

# Plot the original polygon
# Plot the original polygon
x, y = polygon.exterior.xy
plt.plot(x, y, 'b-', label='Original Polygon')

# Initialize the x_seq and y_seq
x_seq = list(x)
y_seq = list(y)

for i in range(1, ring_count + 1):
    inner_polygon = polygon.buffer(i * inset)

    if inner_polygon.is_empty:
        print(f"Cannot create ring {i}, polygon is too small.")
        break
    
    # Extract the coordinates of the inner polygon and reverse them to keep the direction counter-clockwise
    x, y = inner_polygon.exterior.xy
    x = x[::-1]
    y = y[::-1]
    
    # Find the index of the closest point in x, y to the last point in x_seq, y_seq
    min_index = 0
    min_dist = ((x[0] - x_seq[-1]) ** 2 + (y[0] - y_seq[-1]) ** 2) ** 0.5
    for j in range(1, len(x)):
        dist = ((x[j] - x_seq[-1]) ** 2 + (y[j] - y_seq[-1]) ** 2) ** 0.5
        if dist < min_dist:
            min_dist = dist
            min_index = j
    
    # Update x_seq and y_seq by appending the inner polygon points in order starting from the closest point
    x_seq.extend(x[min_index:] + x[:min_index + 1])
    y_seq.extend(y[min_index:] + y[:min_index + 1])

# Plot the continuous line
plt.plot(x_seq, y_seq, 'r-', label='Continuous Line')

# Show the plot
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Polygon with Continuous Line')
plt.legend(loc='upper right')
plt.axis('equal')
plt.show()

