# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/test_dubins_around_circle.py
import dubins
import numpy as np
import matplotlib.pyplot as plt

def plot_circle(ax, center, radius, color='b', linestyle='-', linewidth=2):
    """ Plot a circle on a given axis. """
    circle = plt.Circle(center, radius, color=color, fill=False, linestyle=linestyle, linewidth=linewidth)
    ax.add_patch(circle)

# def sample_path_points(dubins_path, spacing):
#     """ Sample points along the Dubins path at a specified spacing. """
#     sampled_points = []
#     length = dubins_path.path_length()
#     num_samples = int(np.ceil(length / spacing))

#     for i in range(num_samples):
#         sample_output = dubins_path.sample(i * spacing)

#         # Debug prints
#         print(f"Sample output: {sample_output}, Type: {type(sample_output)}")
#         if isinstance(sample_output, (tuple, list)):
#             print(f"Length of sample output: {len(sample_output)}")

#         # Check if the output is a tuple or a list
#         if isinstance(sample_output, (tuple, list)):
#             # Extract the point
#             point = sample_output[0]
#             # Append only the x and y coordinates
#             sampled_points.append(point[:2])
#         else:
#             # If it's a single float, it's likely an error in sampling
#             raise ValueError(f"Unexpected sample output: {sample_output}")

#     return np.array(sampled_points)

def sample_path_points(dubins_path, spacing):
    """ Sample points along the Dubins path at a specified spacing. """
    sampled_points = []
    length = dubins_path.path_length()
    num_samples = int(np.ceil(length / spacing))

    for i in range(num_samples):
        sample_output = dubins_path.sample(i * spacing)

        # Debug prints
        print(f"Sample output: {sample_output}, Type: {type(sample_output)}")
        if isinstance(sample_output, (tuple, list)):
            print(f"Length of sample output: {len(sample_output)}")

        # Check if the output is a tuple or a list
        if isinstance(sample_output, (tuple, list)):
            # Assign directly to point
            point = sample_output
            # Append only the x and y coordinates
            sampled_points.append(point[:2])
        else:
            # If it's a single float, it's likely an error in sampling
            raise ValueError(f"Unexpected sample output: {sample_output}")

    return np.array(sampled_points)

# ... rest of your code ...


# Define your start, waypoint, and end positions
start_position = (1, 1.1, 0.0)  # (x, y, theta)
new_waypoint = (8.0, 1.15, 0)   # Adjusted waypoint with a heading of 0 radians
end_position = (10, 1.2, 0.0)   # (x, y, theta)
turning_radius = 1.3            # Turning radius for the Dubins path

# Create Dubins paths from start to waypoint and from waypoint to end
path1 = dubins.shortest_path(start_position, new_waypoint, turning_radius)
path2 = dubins.shortest_path(new_waypoint, end_position, turning_radius)

# Sample points along the paths at 0.5 meter intervals
sampled_points1 = sample_path_points(path1, 0.5)
sampled_points2 = sample_path_points(path2, 0.5)


# Ensure that both arrays have the same shape
if sampled_points1.shape[1] != sampled_points2.shape[1]:
    if sampled_points1.shape[1] > sampled_points2.shape[1]:
        # Add missing columns with zeros to sampled_points2
        sampled_points2 = np.pad(sampled_points2, ((0, 0), (0, sampled_points1.shape[1] - sampled_points2.shape[1])), 'constant')
    else:
        # Add missing columns with zeros to sampled_points1
        sampled_points1 = np.pad(sampled_points1, ((0, 0), (0, sampled_points2.shape[1] - sampled_points1.shape[1])), 'constant')

# Combine the sampled points
combined_sampled_points = np.vstack((sampled_points1, sampled_points2))

# Plotting
fig, ax = plt.subplots()

# Plot the obstacle (circle)
circle_center = (5, 1.15)  # Center of the obstacle
radius = 2                 # Radius of the obstacle
plot_circle(ax, circle_center, radius)

# Plot the sampled points as dots
ax.plot(combined_sampled_points[:, 0], combined_sampled_points[:, 1], 'g.', markersize=10)

# Mark the start, new waypoint, and end points
ax.plot(start_position[0], start_position[1], 'bo')  # Start
ax.plot(new_waypoint[0], new_waypoint[1], 'ko')  # New Waypoint
ax.plot(end_position[0], end_position[1], 'ro')  # End

# Set labels and title
ax.set_xlabel("X coordinate")
ax.set_ylabel("Y coordinate")
ax.set_title("Sampled Points Along Dubins Path with Dubins Library")
ax.axis('equal')

# Show the plot
plt.show()
