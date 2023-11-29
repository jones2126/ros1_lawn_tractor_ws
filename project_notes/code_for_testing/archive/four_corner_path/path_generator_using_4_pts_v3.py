import math

# Constants for a pentagon
n = 5  # number of sides
area = 80  # area in square meters

# Calculating the side length
angle = math.pi / n
side_length = math.sqrt((4 * area) / (n * math.tan(angle)))

print(f"The side length for a pentagon with an area of 80 square meters is: {side_length}")
