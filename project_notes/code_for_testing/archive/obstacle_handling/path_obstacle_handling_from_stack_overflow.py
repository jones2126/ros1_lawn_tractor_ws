# ref: https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
import numpy as np
def line_circle_intersection_stackoverflow(line_start, line_end, circle_center, circle_radius):
    # Convert inputs to numpy arrays
    A = np.array(line_start)  # Start of the line segment
    B = np.array(line_end)    # End of the line segment
    C = np.array(circle_center)  # Center of the circle
    R = circle_radius

    # Vector from A to B (direction of the ray)
    d = B - A

    # Vector from A to C
    f = A - C

    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - R * R

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        # No intersection
        return []
    else:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        intersections = []
        if 0 <= t1 <= 1:
            intersection_point = A + t1 * d
            intersections.append(tuple(intersection_point))
        if 0 <= t2 <= 1 and not np.array_equal(intersection_point, A + t2 * d):
            intersection_point = A + t2 * d
            intersections.append(tuple(intersection_point))
        
        return intersections

# Test cases
test_cases = [
    ((0, 0), (10, 0), (5, 0), 3),  # Test Case 1
    ((4, 0), (6, 0), (5, 0), 3),   # Test Case 2
    ((0, 3.5), (10, 3.5), (5, 0), 3)  # Test Case 3
]

# Apply the adapted function to each test case
for i, (line_start, line_end, circle_center, circle_radius) in enumerate(test_cases):
    intersections = line_circle_intersection_stackoverflow(line_start, line_end, circle_center, circle_radius)
    print(f"Test Case {i+1} Intersections:", intersections)
