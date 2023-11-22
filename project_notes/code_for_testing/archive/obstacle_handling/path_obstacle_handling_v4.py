
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# def calculate_intersections(A, B, C, circle_radius, circle_center, x1, x2, y1, y2):
#     """
#     Calculate intersection points of a line segment and a circle.

#     Parameters:
#     A, B, C: Coefficients of the line equation Ax + By + C = 0.
#     circle_radius: Radius of the circle.
#     circle_center: (x, y) coordinates of the circle center.
#     x1, x2, y1, y2: Coordinates of the line segment endpoints.

#     Returns:
#     A list of intersection points (tuples).
#     """
#     cx, cy = circle_center
#     a = A*A + B*B
#     b = 2 * (A*C + A*B*cy - B*B*cx)
#     c = C*C + 2*B*C*cy - B*B*(circle_radius**2 + cx**2 + cy**2)

#     discriminant = b*b - 4*a*c
#     if discriminant < 0:
#         return []

#     t1 = (-b + np.sqrt(discriminant)) / (2*a)
#     t2 = (-b - np.sqrt(discriminant)) / (2*a)

#     intersections = []
#     for t in [t1, t2]:
#         if abs(B) > 1e-6:
#             x = t
#             y = (-C - A*x) / B
#         else:
#             x = -C / A
#             y = t

#         if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
#             intersections.append((x, y))

#     return intersections


def calculate_intersections(A, B, C, circle_radius, circle_center, x1, x2, y1, y2):
    cx, cy = circle_center
    a = A * A + B * B
    b = 2 * (A * C + A * B * cy - B * B * cx)
    c = C * C + 2 * B * C * cy + B * B * cx * cx - B * B * (circle_radius**2 + cy * cy)

    discriminant = b * b - 4 * a * c
    if discriminant < 0:
        return []

    discriminant_sqrt = np.sqrt(discriminant)
    t1 = (-b + discriminant_sqrt) / (2 * a)
    t2 = (-b - discriminant_sqrt) / (2 * a)

    intersections = []
    for t in [t1, t2]:
        x = A * t + cx
        y = B * t + cy

        if min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2):
            intersections.append((x, y))

    return intersections



def line_circle_intersection(line_start, line_end, circle_center, circle_radius):
    # Function to calculate line-circle intersections
    x1, y1 = line_start
    x2, y2 = line_end

    # Line equation coefficients: Ax + By + C = 0
    A = y2 - y1
    B = x1 - x2
    C = x2*y1 - x1*y2

    # Calculate intersections
    return calculate_intersections(A, B, C, circle_radius, circle_center, x1, x2, y1, y2)


# Test Case 1: Line Segment Clearly Intersecting the Circle
line_start = (0, 0)
line_end = (10, 0)
circle_center = (5, 0)
circle_radius = 3

intersections = line_circle_intersection(line_start, line_end, circle_center, circle_radius)
print("Test Case 1 Intersections:", intersections)

# Test Case 2: Line Segment Completely Inside the Circle
line_start = (4, 0)
line_end = (6, 0)
circle_center = (5, 0)
circle_radius = 3

intersections = line_circle_intersection(line_start, line_end, circle_center, circle_radius)
print("Test Case 2 Intersections:", intersections)

# Test Case 3: Line Segment Completely Outside and Close to the Circle
line_start = (0, 3.5)
line_end = (10, 3.5)
circle_center = (5, 0)
circle_radius = 3

intersections = line_circle_intersection(line_start, line_end, circle_center, circle_radius)
print("Test Case 3 Intersections:", intersections)
