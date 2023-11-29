import math

# Constants for a pentagon
n = 5  # number of sides
A = 80  # area in square meters

# Calculating the side length
angle = math.pi / n
tan_angle = math.tan(angle)  # tangent of angle
side_length = math.sqrt((4 * A * tan_angle) / n)
print(f"The side length for a pentagon with an area of 80 square meters is: {side_length}")
