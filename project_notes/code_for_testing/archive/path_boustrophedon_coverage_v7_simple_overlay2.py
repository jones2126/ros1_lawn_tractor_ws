
from shapely.geometry import LineString, Polygon
from shapely.affinity import rotate
from statistics import mean
import numpy as np
import matplotlib.pyplot as plt

# Function to generate and trim lines
def generate_and_trim_lines(polygon_points, separation, angle):
    polygon = Polygon(polygon_points)
    centroid = (mean([p[0] for p in polygon_points]), mean([p[1] for p in polygon_points]))
    x_coords, y_coords = zip(*polygon_points)
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    width = max_x - min_x
    height = max_y - min_y
    diagonal_length = np.sqrt(width**2 + height**2) * np.sqrt(2)
    num_lines = int(np.ceil(diagonal_length / separation)) + 1
    
    half_length = diagonal_length / 2
    start_x = centroid[0] - half_length
    start_y = centroid[1]
    
    lines = []
    for i in range(num_lines):
        offset = (i - num_lines // 2) * separation
        line_start = (start_x, start_y + offset)
        line_end = (start_x + diagonal_length, start_y + offset)
        line = LineString([line_start, line_end])
        line = rotate(line, angle, origin=centroid)
        lines.append(line)
    
    trimmed_lines = [line.intersection(polygon) for line in lines if not line.is_empty]
    return trimmed_lines, polygon, centroid

# Function for plotting
def plot_polygon_and_lines(polygon_points, separation, angle):
    trimmed_lines, polygon, centroid = generate_and_trim_lines(polygon_points, separation, angle)
    plt.figure(figsize=(10, 10))
    x, y = polygon.exterior.xy
    plt.plot(x, y, color="red", alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2, label='Polygon')

    for line in trimmed_lines:
        if not line.is_empty:
            x, y = line.xy
            plt.plot(x, y, color="blue", linewidth=1, alpha=0.7)

    plt.plot(centroid[0], centroid[1], 'ko', label='Centroid')
    plt.title(f"Polygon with Trimmed Line Segments at {angle} Degrees")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# Polygon points provided by the user
polygon_points = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]

# Call the plot function with an angle of 75 degrees
plot_polygon_and_lines(polygon_points, separation=0.9, angle=75)
