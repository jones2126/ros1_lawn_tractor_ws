
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage_v7_simple_overlay6.py
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate
from statistics import mean
import numpy as np

# Set the function to use for writing warnings
import warnings
def write_warning_to_file(message, category, filename, lineno, file=None, line=None):
    with open("/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/warnings_from_path_boustrophedon.txt", "a") as f:  
        f.write(f"{category.__name__}:{filename}:{lineno}:{message}\n")
warnings.showwarning = write_warning_to_file

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

def calculate_dimensions(polygon_points):
    width = max(p[0] for p in polygon_points) - min(p[0] for p in polygon_points)
    height = max(p[1] for p in polygon_points) - min(p[1] for p in polygon_points)
    return width, height

# Make sure the following function returns both the trimmed lines and their coordinates

# def trim_and_filter_lines(lines, polygon):
#     trimmed_lines = []
#     line_coordinates = []  # List to store coordinates of trimmed lines

#     for line in lines:
#         if line.is_valid and not line.is_empty:
#             intersection = line.intersection(polygon)
#             if not intersection.is_empty:
#                 # Check if the intersection is a LineString
#                 if isinstance(intersection, LineString):
#                     trimmed_lines.append(intersection)
#                     coords = list(intersection.coords)
#                     line_coordinates.append(coords)
#                 # Check if the intersection is a MultiLineString
#                 elif isinstance(intersection, MultiLineString):
#                     for geom in intersection.geoms:  # Corrected line to use .geoms
#                         trimmed_lines.append(geom)
#                         coords = list(geom.coords)
#                         line_coordinates.append(coords)
#                 else:
#                     # Handle other types of intersections or raise an error
#                     print(f"Unhandled geometry type: {type(intersection)}")
#     print(f"Lines trimmed and filtered, remaining lines: {len(trimmed_lines)}")
#     return trimmed_lines, line_coordinates

# def trim_and_filter_lines(lines, polygon):
#     trimmed_lines = []
#     line_coordinates = []
#     for line in lines:
#         if line.is_valid and not line.is_empty:
#             intersection = line.intersection(polygon)
#             if isinstance(intersection, LineString):
#                 trimmed_lines.append(intersection)
#                 line_coordinates.append(list(intersection.coords))
#             elif isinstance(intersection, MultiLineString):
#                 # Detected a MultiLineString indicating disconnected paths
#                 print("A disconnected path (MultiLineString) was detected.")
#                 for geom in intersection.geoms:
#                     trimmed_lines.append(geom)
#                     line_coordinates.append(list(geom.coords))
#             else:
#                 print(f"Unhandled geometry type: {type(intersection)}")

#     print(f"Lines trimmed and filtered, remaining lines: {len(trimmed_lines)}")
#     return trimmed_lines, line_coordinates


def trim_and_filter_lines(lines, polygon):
    trimmed_lines = []
    line_coordinates = []

    for line in lines:
        if line.is_valid and not line.is_empty:
            intersection = line.intersection(polygon)

            # Process only LineStrings and non-empty MultiLineStrings
            if isinstance(intersection, LineString) and not intersection.is_empty:
                trimmed_lines.append(intersection)
                line_coordinates.append(list(intersection.coords))
            elif isinstance(intersection, MultiLineString):
                non_empty_geoms = [geom for geom in intersection.geoms if not geom.is_empty]
                if non_empty_geoms:
                    print("A disconnected path (MultiLineString) was detected.")
                    for geom in non_empty_geoms:
                        trimmed_lines.append(geom)
                        line_coordinates.append(list(geom.coords))

    # Remove any empty coordinate lists
    line_coordinates = [coords for coords in line_coordinates if coords]

    print(f"Lines trimmed and filtered, remaining lines: {len(trimmed_lines)}")
    return trimmed_lines, line_coordinates

# Function to calculate the total length of line segments
def calculate_total_length(line_coordinates):
    from shapely.geometry import LineString
    total_length = sum(LineString(segment).length for segment in line_coordinates if segment)
    return total_length

# Function to check for multipaths
def check_for_multipath(trimmed_lines):
    # Implement your logic to check for multi-path instances
    # For now, this is a placeholder function
    return any(isinstance(line, MultiLineString) for line in trimmed_lines)

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
    
    trimmed_lines = [line.intersection(polygon) for line in lines if line.is_valid and not line.is_empty and polygon.is_valid]
    return trimmed_lines, polygon, centroid

def extract_line_coordinates(trimmed_lines):
    line_coordinates = []
    for line in trimmed_lines:
        if isinstance(line, LineString):
            coords = list(line.coords)
            line_coordinates.append(coords)
        elif isinstance(line, MultiLineString):
            for segment in line:
                coords = list(segment.coords)
                line_coordinates.append(coords)
    print("Extracted line coordinates.")
    return line_coordinates


# def main(polygon_points, separation, angle):
#     polygon, valid = create_polygon(polygon_points)
#     if not valid:
#         return "Invalid polygon."
#     centroid = calculate_polygon_center(polygon_points)
#     width, height = calculate_dimensions(polygon_points)
#     lines = build_grid_of_lines(centroid, width, height, separation, angle)
#     trimmed_lines, line_coordinates = trim_and_filter_lines(lines, polygon)  # This function should return both lines and coordinates
#     plot_polygon_and_lines(polygon, trimmed_lines)  # Pass only the LineString objects for plotting
#     return trimmed_lines, line_coordinates, polygon, centroid  # Return all relevant results

def main(polygon_points, separation):
    summary_of_segments = []
    for angle in range(0, 91, 10):
        print(f"angle: {angle}")
        trimmed_lines, polygon, centroid = generate_and_trim_lines(polygon_points, separation, angle)
        line_coordinates = extract_line_coordinates(trimmed_lines)
        total_length = calculate_total_length(line_coordinates)
        multipath = check_for_multipath(trimmed_lines)
        summary_of_segments.append((angle, len(line_coordinates), total_length, multipath))
    print(summary_of_segments)




# Test the functions with provided polygon points
polygon_points = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]
separation = 0.9
trimmed_lines, line_coordinates, polygon, centroid = main(polygon_points, separation)
print(f"EOJ....")