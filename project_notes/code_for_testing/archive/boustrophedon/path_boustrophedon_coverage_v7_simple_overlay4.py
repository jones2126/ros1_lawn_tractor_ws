from shapely.geometry import Polygon, LineString
from shapely.affinity import rotate
from statistics import mean
import numpy as np

# import warnings

# with warnings.catch_warnings():
#     warnings.filterwarnings("ignore", category=RuntimeWarning)

# Function to create the polygon and test its validity
def create_polygon(polygon_points):
    polygon = Polygon(polygon_points)
    valid = polygon.is_valid
    print(f"Polygon created. Validity: {valid}")
    return polygon, valid

# Function to calculate the polygon's center
def calculate_polygon_center(polygon_points):
    centroid = (mean([p[0] for p in polygon_points]), mean([p[1] for p in polygon_points]))
    print(f"Polygon center calculated at: {centroid}")
    return centroid

# Function to build a grid of lines
def build_grid_of_lines(centroid, width, height, separation, angle):
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
        
    print(f"Grid of {num_lines} lines built.")
    return lines

# # Function to overlay the grid of lines on the polygon
# def overlay_grid_on_polygon(lines, polygon):
#     # This function doesn't perform any operations yet, just a placeholder
#     print("Grid overlay completed.")
#     return lines

def trim_and_filter_lines(lines, polygon):
    trimmed_lines = []
    line_coordinates = []  # List to store coordinates of trimmed lines

    for line in lines:
        if line.is_valid and not line.is_empty:
            try:
                intersection = line.intersection(polygon)
                if not intersection.is_empty:
                    trimmed_lines.append(intersection)
                    # Extract the x and y coordinates of the trimmed line
                    coords = list(intersection.coords)
                    line_coordinates.append(coords)
            except Exception as e:
                print(f"An error occurred during intersection: {e}")
    print(f"Lines trimmed and filtered, remaining lines: {len(trimmed_lines)}")
    return trimmed_lines, line_coordinates

# Function to plot the polygon and lines
def plot_polygon_and_lines(polygon, lines):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots()
    x, y = polygon.exterior.xy
    ax.plot(x, y, color="red", alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)

    for line in lines:
        if not line.is_empty:
            x, y = line.xy
            ax.plot(x, y, color="blue", linewidth=1, alpha=0.7)

    ax.set_title("Polygon with Trimmed Line Segments")
    ax.set_aspect('equal', 'box')
    plt.axis('equal')
    plt.show()    

# Now, let's define the main function that uses these helper functions
def main(polygon_points, separation, angle):
    polygon, valid = create_polygon(polygon_points)
    if not valid:
        return "Invalid polygon."
    
    centroid = calculate_polygon_center(polygon_points)
    width = max(p[0] for p in polygon_points) - min(p[0] for p in polygon_points)
    height = max(p[1] for p in polygon_points) - min(p[1] for p in polygon_points)
    
    lines = build_grid_of_lines(centroid, width, height, separation, angle)
    #overlay_grid_on_polygon(lines, polygon)
    trimmed_lines = trim_and_filter_lines(lines, polygon)
    plot_polygon_and_lines(polygon, trimmed_lines)
    
    return trimmed_lines, polygon, centroid

# Test the functions with provided polygon points
polygon_points_test = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]
separation_test = 0.9
angle_test = 40
print(f"angle_test: {angle_test}")
# Execute the main function
result = main(polygon_points_test, separation_test, angle_test)
#print(result)
