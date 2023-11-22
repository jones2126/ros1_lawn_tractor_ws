# import matplotlib.pyplot as plt
# import numpy as np

# # Circle parameters
# center_x, center_y = 17.6, -9.5
# radius = 5.4 / 2

# # Generating points at 30 degree intervals
# angles = np.arange(0, 360, 30)  # Degrees from 0 to 330
# points_x = center_x + radius * np.cos(np.radians(angles))
# points_y = center_y + radius * np.sin(np.radians(angles))
# print("angles:", angles)

# # Plotting
# plt.figure(figsize=(8, 8))
# circle = plt.Circle((center_x, center_y), radius, color='blue', fill=False)
# plt.gca().add_patch(circle)
# plt.scatter(points_x, points_y, color='red')

# # Annotating points
# for x, y in zip(points_x, points_y):
#     plt.annotate(f'({x:.2f}, {y:.2f})', (x, y), textcoords="offset points", xytext=(0,10), ha='center')

# plt.xlim(center_x - radius - 1, center_x + radius + 1)
# plt.ylim(center_y - radius - 1, center_y + radius + 1)
# plt.gca().set_aspect('equal', adjustable='box')


# plt.title("Circle with Points at 30° Intervals")
# plt.show()
import matplotlib.pyplot as plt
import numpy as np

# Circle parameters
center_x, center_y = 17.6, -9.5
radius = 5.4 / 2

# Generating points at 30 degree intervals
angles = np.arange(0, 360, 30)  # Degrees from 0 to 330
points_x = center_x + radius * np.cos(np.radians(angles))
points_y = center_y + radius * np.sin(np.radians(angles))

# Initialize a list to store coordinates
coordinates = []

# Plotting
plt.figure(figsize=(8, 8))
circle = plt.Circle((center_x, center_y), radius, color='blue', fill=False)
plt.gca().add_patch(circle)
plt.scatter(points_x, points_y, color='red')

# Annotating points and adding them to the list
for x, y in zip(points_x, points_y):
    plt.annotate(f'({x:.2f}, {y:.2f})', (x, y), textcoords="offset points", xytext=(0,10), ha='center')
    coordinates.append((round(x, 2), round(y, 2)))

plt.xlim(center_x - radius - 1, center_x + radius + 1)
plt.ylim(center_y - radius - 1, center_y + radius + 1)
plt.gca().set_aspect('equal', adjustable='box')

plt.title("Circle with Points at 30° Intervals")
plt.show()

# Printing the list of coordinates
# print("Coordinates of points on the circle:")
# for coord in coordinates:
#     print(coord)
# Define the starting position
# position_to_start = 1

# # Print the coordinates starting from 'position_to_start'
# print("Coordinates of points on the circle starting from position", position_to_start, ":")
# for coord in coordinates[position_to_start:] + coordinates[:position_to_start]:
#     print(coord)

# Direction and starting position
direction = 'clockwise'  # 'clockwise' or 'counter-clockwise'
position_to_start = 1

# Reversing the list if direction is clockwise
if direction == 'clockwise':
    coordinates = coordinates[::-1]

# Calculating the new starting position after reversal
new_start = len(coordinates) - position_to_start if direction == 'clockwise' else position_to_start

# Print the coordinates starting from 'position_to_start'
print(f"Coordinates of points on the circle starting from position {new_start}, {direction}:")
for coord in coordinates[new_start:] + coordinates[:new_start]:
    print(coord)