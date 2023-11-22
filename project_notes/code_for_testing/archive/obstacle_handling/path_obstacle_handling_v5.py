import numpy as np

def line_circle_intersection(line_start, line_end, circle_center, circle_radius):
    # Convert inputs to numpy arrays for easier calculations
    A = np.array(line_start)
    B = np.array(line_end)
    C = np.array(circle_center)
    R = circle_radius

    # Vector AB
    AB = B - A

    # Project vector AC onto AB to find the closest point D on AB to the circle center C
    AC = C - A
    t = np.dot(AC, AB) / np.dot(AB, AB)
    D = A + t * AB

    # Calculate the distance from D to C
    DC = np.linalg.norm(D - C)

    # If the distance from D to C is greater than the radius, there's no intersection
    if DC > R:
        return []

    # Find the distance from D to the intersection points along AB
    dt = np.sqrt(R**2 - DC**2) / np.linalg.norm(AB)

    # Calculate the intersection points
    intersection1 = D - dt * AB
    intersection2 = D + dt * AB

    # Check if the intersections are within the line segment
    intersections = []
    for P in [intersection1, intersection2]:
        t = np.dot(P - A, AB) / np.dot(AB, AB)
        if 0 <= t <= 1:
            intersections.append(tuple(P))

    return intersections

# Test cases
test_cases = [
    ((0, 0), (10, 0), (5, 0), 3),  # Test Case 1
    ((4, 0), (6, 0), (5, 0), 3),   # Test Case 2
    ((0, 3.5), (10, 3.5), (5, 0), 3)  # Test Case 3
]

# Apply the function to each test case
for i, (line_start, line_end, circle_center, circle_radius) in enumerate(test_cases):
    intersections = line_circle_intersection(line_start, line_end, circle_center, circle_radius)
    print(f"Test Case {i+1} Intersections:", intersections)

