# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/obstacle_handling/calc_circle_tangent.py
import numpy as np
import matplotlib.pyplot as plt

def calculate_tangent_point(circle_center, radius, external_point):
    """
    Calculate the tangent point on the circle from an external point (i.e. The points on the circle that are tangent to lines 
    drawn from the start and end points outside the circle.)  This means these tangent points are the points on the circle 
    where a line from either the start or end point touches the circle at exactly one point, forming a right angle with 
    the circle's radius at that point.

    :param circle_center: The center of the circle (x, y)
    :param radius: The radius of the circle
    :param external_point: The external point from which the tangent is drawn (x, y)
    :return: The tangent point on the circle (x, y)
    """
    # Extract coordinates
    cx, cy = circle_center
    ex, ey = external_point

    # Calculate the distance from the external point to the circle center
    distance = np.sqrt((ex - cx)**2 + (ey - cy)**2)

    # Calculate the angle from the circle center to the external point
    angle_to_external = np.arctan2(ey - cy, ex - cx)

    # Calculate the angle of the tangent from the radius to the point of tangency
    tangent_angle = np.arcsin(radius / distance)

    # Calculate the angles for the tangent points
    tangent_point_angle1 = angle_to_external + tangent_angle
    tangent_point_angle2 = angle_to_external - tangent_angle

    # Calculate the tangent points
    tangent_point1 = (cx + radius * np.cos(tangent_point_angle1), cy + radius * np.sin(tangent_point_angle1))
    tangent_point2 = (cx + radius * np.cos(tangent_point_angle2), cy + radius * np.sin(tangent_point_angle2))

    return tangent_point1, tangent_point2

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

def plot_scenario(circle_center, radius, start_point, end_point, tangent_points_start, tangent_points_end, intersections):
    fig, ax = plt.subplots()

    # Plot the circle
    circle = plt.Circle(circle_center, radius, color='blue', fill=False)
    ax.add_artist(circle)

    # Plot the line segment from start to end
    ax.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], 'k-', label='Start-End Line')

    # Plot the tangent points for start and end
    ax.plot(*tangent_points_start[0], 'ro', label='Tangent Point Start 1')
    ax.plot(*tangent_points_start[1], 'go', label='Tangent Point Start 2')
    ax.plot(*tangent_points_end[0], 'yo', label='Tangent Point End 1')
    ax.plot(*tangent_points_end[1], 'mo', label='Tangent Point End 2')

    # Plot the intersection points
    for i, point in enumerate(intersections):
        ax.plot(*point, 'cx', label=f'Intersection Point {i+1}')

    # Setting the plot limits
    ax.set_xlim(0, 11)
    ax.set_ylim(-3, 5)

    # Make axes equal
    ax.set_aspect('equal', adjustable='box')

    # Adding labels and legend
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.legend()

    # Show plot
    plt.show()


# Define the obstacle (circle) and the start and end points
circle_center = (5, 1.15)
radius = 2
start_point = (1, 1.1)
end_point = (10, 1.2)

# Calculate the tangent points for the start and end points
tangent_points_start = calculate_tangent_point(circle_center, radius, start_point)
tangent_points_end = calculate_tangent_point(circle_center, radius, end_point)

intersections = find_line_circle_intersections(circle_center, radius, start_point, end_point)

#print("tangent_points_start:", tangent_points_start, "tangent_points_end:", tangent_points_end)

# Call the plotting function
plot_scenario(circle_center, radius, start_point, end_point, tangent_points_start, tangent_points_end, intersections)