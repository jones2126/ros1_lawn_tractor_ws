
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage_v6_using_openmower_logic.py
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely.affinity import rotate
import matplotlib.pyplot as plt

# Simulating the create_mowing_plan functionality in Python.
def create_mowing_plan_python1(points, line_spacing=0.9, angle=0):
    # Create a polygon from the provided points
    polygon = Polygon(points)
    
    # Rotate the polygon to align with the specified angle
    rotated_polygon = rotate(polygon, angle, origin='centroid', use_radians=True)
    
    # Get the bounds of the rotated polygon
    minx, miny, maxx, maxy = rotated_polygon.bounds
    
    # Determine the number of lines based on the line spacing
    num_lines = int(np.ceil((maxx - minx) / line_spacing))
    
    # Create the mowing paths (Boustrophedon pattern)
    mowing_paths = []
    for i in range(num_lines + 1):
        x = minx + i * line_spacing
        line = LineString([(x, miny), (x, maxy)])
        # Check intersection with the rotated polygon
        intersection = rotated_polygon.intersection(line)
        if not intersection.is_empty:
            # Rotate the line back to the original polygon's orientation
            original_line = rotate(intersection, -angle, origin='centroid', use_radians=True)
            mowing_paths.append(original_line)
    
    return mowing_paths

def create_mowing_plan_python2(points, line_spacing=0.9, angle=45):
    # Create a polygon from the provided points
    polygon = Polygon(points)
    
    # Rotate the polygon to align the slicing lines with the x-axis at 45 degrees
    rotated_polygon = rotate(polygon, angle, origin='centroid', use_radians=False)
    minx, miny, maxx, maxy = rotated_polygon.bounds

    # Determine the number of lines based on the line spacing and the diagonal length of the bounds
    diag_length = np.sqrt((maxx - minx)**2 + (maxy - miny)**2)
    num_lines = int(np.floor(diag_length / line_spacing))

    # Create the mowing paths
    mowing_paths = []
    for i in range(num_lines + 1):
        # Generate a line at the current position and rotate it to 45 degrees
        x_offset = (line_spacing * i) / np.sqrt(2)  # offset by 45 degrees
        line = LineString([(minx - x_offset, miny), (maxx, maxy - x_offset)])
        # Find the intersection of the line with the polygon
        intersection = rotated_polygon.intersection(line)
        if not intersection.is_empty:
            # Rotate the line back to the original orientation
            original_line = rotate(intersection, -angle, origin='centroid', use_radians=False)
            mowing_paths.append(original_line)

    return mowing_paths    

def plot_mowing_plan(polygon, mowing_paths):
    # Plot the polygon
    x, y = polygon.exterior.xy
    plt.plot(x, y, color='blue', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
    
    # Plot the mowing paths
    for path in mowing_paths:
        x, y = path.xy
        plt.plot(x, y, color='red', linewidth=2, alpha=0.7, zorder=3)
    
    plt.title('Mowing Plan')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Define the polygon points
polygon_points = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]

# Create a mowing plan based on the provided polygon
mowing_paths1 = create_mowing_plan_python1(polygon_points, line_spacing=0.9)
# Plot the mowing plan
polygon = Polygon(polygon_points)
plot_mowing_plan(polygon, mowing_paths1)
# Create a mowing plan based on the provided polygon with lines at 45 degrees (pi/4 radians)
mowing_paths2 = create_mowing_plan_python2(polygon_points, line_spacing=0.9, angle=45)
# Plot the mowing plan
polygon = Polygon(polygon_points)
plot_mowing_plan(polygon, mowing_paths2)
