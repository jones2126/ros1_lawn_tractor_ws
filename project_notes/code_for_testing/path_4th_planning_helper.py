#!/usr/bin/env python
'''
Script that helps build a path to follow using four corner inputs

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_4th_planning_helper.py

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

# Create a polygon - counter clockwise
polygon = Polygon([
    (lower_right_x, lower_right_y),
    (upper_right_x, upper_right_y),
    (upper_left_x, upper_left_y),
    (lower_left_x, lower_left_y)
])

# The distance for the inner rings
inset = -1.0  # negative for inside buffering

# Loop to create inner polygons and print their coordinates
ring_count = 4  # adjust as needed to create the number of rings you want

def calculate_angle_from_origin(x, y):
    # Calculating the angle between the x-axis and a 2D vector starting from the origin
    return math.atan2(y, x)


# Loop to create inner polygons and store their coordinates in a list along with the ring number and angle

# Loop to create inner polygons and store their coordinates in a list along with the ring number and angle
coordinates_list = []
for i in range(1, ring_count + 1):
    inner_polygon = polygon.buffer(i * inset)

    if inner_polygon.is_empty:
        print(f"Cannot create ring {i}, polygon is too small.")
        break
    sequence = 0
    x, y = inner_polygon.exterior.xy
    for xi, yi in zip(x[:-1], y[:-1]):
    	sequence += 1
    	# angle = calculate_angle_from_origin(xi, yi)
    	coordinates_list.append((i, sequence, xi, yi))

# Loop to change the sequnce of the points so they are in a counter clockwise rotation
adjusted_coordinates_list = []
for i, sequence, xi, yi in coordinates_list:
    if sequence == 2:
        sequence = 4
    elif sequence == 4:
        sequence = 2
    adjusted_coordinates_list.append((i, sequence, xi, yi))

# Defining the origin coordinate
origin = (0, 0, 7.7, -3.0)
adjusted_coordinates_list.insert(0, origin)

# Sort the list by ring number and adjusted sequence
sorted_coordinates_list = sorted(adjusted_coordinates_list, key=lambda t: (t[0], t[1]))

print("Adjusted List:")
for coord in sorted_coordinates_list:
    print(coord)

# Print out the sorted coordinates
# current_ring = 1
# print(f"\nSorted Coordinates:")
# print(f"Ring {current_ring} Coordinates:")
# for ring, sequence, xi, yi in sorted_coordinates_list:
#     if ring != current_ring:
#         current_ring = ring
#         print(f"Ring {ring} Coordinates:")
#     print(xi, yi)
# Initialize a new list to store extended tuples
extended_coordinates_list = []

# Iterate over sorted_coordinates_list and calculate the angle for each coordinate
list_len = len(sorted_coordinates_list)
for i in range(list_len):
    # Get the current coordinate and the next coordinate in the list
    ring1, seq1, x1, y1 = sorted_coordinates_list[i]
    ring2, seq2, x2, y2 = sorted_coordinates_list[(i + 1) % list_len]  # Using modulo to loop back to the start of the list
    
    # Calculate the angle between the current and the next coordinate
    angle = calculate_angle(x1, y1, x2, y2)
    adjusted_angle = adjust_angle(angle)
    # Append the extended tuple to the new list
    extended_coordinates_list.append((ring1, seq1, x1, y1, adjusted_angle))

# Now, extended_coordinates_list contains the sorted coordinates along with their corresponding angles.
# Print out the extended_coordinates_list
# for coord in extended_coordinates_list:
#     print(coord)

# Open the file with write ('w') permission. 
# If the file doesn't exist, it will be created; if it exists, it will be truncated.
with open('output.txt', 'w') as file:
    # Loop through each coordinate in the extended_coordinates_list
    for _, _, x, y, angle in extended_coordinates_list:
        # Write x, y, and angle to the file separated by a space and add a newline at the end
        file.write(f"{x} {y} {angle}\n")


# Extracting the x and y coordinates from the extended_coordinates_list
x_coords = [x for _, _, x, _, _ in extended_coordinates_list]
y_coords = [y for _, _, _, y, _ in extended_coordinates_list]

# Adding the first point at the end to close the polygon
x_coords.append(extended_coordinates_list[0][2])
y_coords.append(extended_coordinates_list[0][3])

# Create a figure and axis
fig, ax = plt.subplots()

# Plot the points
ax.scatter(x_coords, y_coords, c='red')

# Annotate the points with their sequence
for coord in extended_coordinates_list:
    ax.text(coord[2], coord[3], f"{coord[1]}")

# Connect the points with lines
ax.plot(x_coords, y_coords, 'b-')

# Show the plot
plt.show()
