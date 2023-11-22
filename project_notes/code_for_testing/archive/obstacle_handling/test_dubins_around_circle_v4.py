# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/test_dubins_around_circle_v4.py
import dubins
import numpy as np
import matplotlib.pyplot as plt

def plot_circle(ax, center, radius, color='b', linestyle='-', linewidth=2):
    """ Plot a circle on a given axis. """
    circle = plt.Circle(center, radius, color=color, fill=False, linestyle=linestyle, linewidth=linewidth)
    ax.add_patch(circle)

def sample_path_points(dubins_path, spacing):
    """ Sample points along the Dubins path at a specified spacing. """
    sampled_points = []
    length = dubins_path.path_length()
    num_samples = int(np.ceil(length / spacing))

    for i in range(num_samples):
        sample_output = dubins_path.sample(i * spacing)

        # Debug prints
        #print(f"Sample output: {sample_output}, Type: {type(sample_output)}")
        #if isinstance(sample_output, (tuple, list)):
            #print(f"Length of sample output: {len(sample_output)}")

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

def find_line_circle_intersections(circle_center, radius, start_point, end_point):
    h, k = circle_center
    r = radius

    # Calculate the coefficients of the line (y = mx + b)
    if end_point[0] != start_point[0]:  # Non-vertical line
        m = (end_point[1] - start_point[1]) / (end_point[0] - start_point[0])
        b = start_point[1] - m * start_point[0]

        # Quadratic equation coefficients (Ax^2 + Bx + C = 0)
        A = 1 + m**2
        B = -2*h + 2*m*(b - k)
        C = h**2 + (b - k)**2 - r**2

        # Solving the quadratic equation for x
        discriminant = B**2 - 4*A*C
        if discriminant < 0:  # No real intersections
            return []
        else:
            x1 = (-B + np.sqrt(discriminant)) / (2*A)
            x2 = (-B - np.sqrt(discriminant)) / (2*A)
            y1 = m*x1 + b
            y2 = m*x2 + b
            return [(x1, y1), (x2, y2)]
    else:  # Vertical line
        x = start_point[0]
        # Circle equation reduced to (y - k)^2 = r^2 - (x - h)^2
        discriminant = r**2 - (x - h)**2
        if discriminant < 0:  # No real intersections
            return []
        else:
            y1 = k + np.sqrt(discriminant)
            y2 = k - np.sqrt(discriminant)
            return [(x, y1), (x, y2)]

def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def angle_between_points(center, point):
    return np.arctan2(point[1] - center[1], point[0] - center[0])

def plot_path_and_obstacle(circle_center, radius, full_path):
    print("circle_center:", circle_center)
    fig, ax = plt.subplots()

    # Plot the circle (obstacle)
    circle = plt.Circle(circle_center, radius, color='blue', fill=False)
    ax.add_artist(circle)

    # Plot the full path
    # Extracting x and y coordinates from the path points
    x_coords = [point[0] for point in full_path]
    y_coords = [point[1] for point in full_path]
    ax.plot(x_coords, y_coords, 'r-', label='Path')

    # Mark start and end points
    ax.plot(full_path[0][0], full_path[0][1], 'go', label='Start')
    ax.plot(full_path[-1][0], full_path[-1][1], 'mo', label='End')

    # # Setting plot limits for better visualization
    # ax.set_xlim(min(x_coords) - 1, max(x_coords) + 1)
    # ax.set_ylim(min(y_coords) - 1, max(y_coords) + 1)

    # Setting the plot limits
    ax.set_xlim(0, 11)
    ax.set_ylim(-3, 5)    

    # Make axes equal to maintain aspect ratio
    #ax.set_aspect('equal', adjustable='box')
    ax.set_aspect('equal')    

    # Adding labels and legend
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.legend()

    # Show plot
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
def calculate_shortest_path(circle_center, radius, start_point, end_point, num_points=20):
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


# Define your start, waypoint, and end positions

# start_position = (1, 1.1, 0.0)  # (x, y, theta)
# end_position = (10, 1.2, 0.0)   # (x, y, theta)

end_position = (1, 1.1, 3.14)  # (x, y, theta)
start_position = (10, 1.2, 3.14)   # (x, y, theta)
turning_radius = 1.3            # Turning radius for the Dubins path

# Create Dubins paths from start to waypoint and from waypoint to end
path1 = dubins.shortest_path(start_position, end_position, turning_radius)
#print("path1:", path1)
#path2 = dubins.shortest_path(new_waypoint, end_position, turning_radius)

# Sample points along the paths at 0.5 meter intervals
sampled_points1 = sample_path_points(path1, 0.5)
#print("sample_points1", sampled_points1)

combined_sampled_points = sampled_points1

# Plotting
fig, ax = plt.subplots()

# Plot the obstacle (circle)
circle_center = (5, 2.9)  # Center of the obstacle
radius = 2                 # Radius of the obstacle
plot_circle(ax, circle_center, radius)

# Plot the sampled points as dots
ax.plot(combined_sampled_points[:, 0], combined_sampled_points[:, 1], 'g.', markersize=10)

# Mark the start, new waypoint, and end points
ax.plot(start_position[0], start_position[1], 'bo')  # Start
#ax.plot(new_waypoint[0], new_waypoint[1], 'ko')  # New Waypoint
ax.plot(end_position[0], end_position[1], 'ro')  # End

# Set labels and title
ax.set_xlabel("X coordinate")
ax.set_ylabel("Y coordinate")
ax.set_title("Sampled Points Along Dubins Path with Dubins Library")
ax.axis('equal')

# Show the plot
plt.show()

# calculate the intersection points and plot a path from the start to the first intersection point, around the circle at no more 
# than .3 meter points, to the second intersection point and then to the end point.
x, y, _ = start_position
start_point = (x, y) 
x, y, _ = end_position
end_point = (x, y) 
intersections = find_line_circle_intersections(circle_center, radius, start_point, end_point)
if len(intersections) == 2:
    print(intersections)
    # find the closest intersection point; Make the path the first point and the first intersection point; then the circle

    # Sort intersections based on distance from start_point so we are working with the correct side of the circle
    intersections = sorted(intersections, key=lambda p: np.hypot(p[0]-start_point[0], p[1]-start_point[1]))
    closest_point = intersections[0]
    farthest_point = intersections[1]


    #print("Closest intersection point:", closest_point)
    #print("farthest_point:", farthest_point)
    #print("[start_point, closest_point]", [start_point, closest_point])
    #print("[farthest_point, end_point]", [farthest_point, end_point])
    #circle_path = calculate_shortest_circle_path(circle_center, radius, start_point, end_point, num_points=20)

    circle_arc = calculate_shortest_path(circle_center, radius, closest_point, farthest_point, num_points=10)

    #full_path = [start_point] + circle_arc + [end_point]
    full_path = [start_point, closest_point] + circle_arc + [farthest_point, end_point]
    #print("circle_arc", circle_arc)
else:
    # Direct path from start to end as no intersections are found
    full_path = [start_point, end_point]    
plot_path_and_obstacle(circle_center, radius, full_path)
