# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage_47_degree.py

import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate
from matplotlib import pyplot as plt, patheffects

def boustrophedon_path_corrected(polygon, line_spacing, angle_degrees):
    # Normalize the angle to be within -90 to 90 degrees for intersection purposes
    angle_degrees = angle_degrees % 180  # Limit the angle within the 0-180 degree range
    if angle_degrees > 90:
        angle_degrees -= 180  # Convert 91-180 degrees to -89 to -1 degrees

    # Convert the angle from degrees to radians
    angle_radians = np.radians(angle_degrees)
    
    # Rotate the polygon to align the slicing lines with the y-axis
    rotated_polygon = rotate(polygon, -angle_degrees, origin='centroid', use_radians=False)
    minx, miny, maxx, maxy = rotated_polygon.bounds

    # Calculate the length needed for lines to cover the entire area after rotation
    bounding_width = maxx - minx
    bounding_height = maxy - miny
    diag_length = np.sqrt(bounding_width**2 + bounding_height**2)

    # Determine the extended bounds for the lines to ensure they cover the entire rotated polygon area
    extended_bounds = diag_length * np.sqrt(2)
    extended_minx = minx - extended_bounds
    extended_maxx = maxx + extended_bounds
    extended_miny = miny - extended_bounds
    extended_maxy = maxy + extended_bounds

    # Calculate the number of lines needed, considering the line spacing
    num_lines = int(np.ceil(extended_bounds * 2 / (line_spacing * np.cos(angle_radians))))

    path = []
    # Generate lines from one side of the bounding box to the other
    for i in range(num_lines + 1):
        # Calculate the start point of the line
        startx = extended_minx + (line_spacing * np.cos(angle_radians)) * i
        starty = extended_miny
        endx = startx
        endy = extended_maxy
        
        # Create a line that will be rotated to the specified angle
        line = LineString([(startx, starty), (endx, endy)])
        # Rotate the line to the specified angle
        rotated_line = rotate(line, angle_degrees, origin='centroid', use_radians=False)
        # Find the intersection of the rotated line with the original polygon
        intersection = polygon.intersection(rotated_line)
        if not intersection.is_empty:
            # Only consider LineStrings or MultiLineString (ignore Points)
            if isinstance(intersection, LineString):
                path.append(intersection)
            elif isinstance(intersection, MultiLineString):

                # for line in intersection:
                #     path.append(line)

                for line in intersection.geoms:
                    path.append(line)                    


    # Count the total number of line segments excluding the joining segments
    return path, len(path)

# The plot_path function remains the same, now we call it plot_path_corrected
def plot_path(polygon, path_lines, degree):
    fig, ax = plt.subplots()
    x, y = polygon.exterior.xy
    ax.plot(x, y, color='blue', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)

    for idx, line in enumerate(path_lines):
        x, y = line.xy
        ax.plot(x, y, color='red', linewidth=2, alpha=0.7, zorder=2)
        # Position the text in the center of the line segment
        text_x = (x[0] + x[1]) / 2
        text_y = (y[0] + y[1]) / 2
        #ax.text(text_x, text_y, str(idx + 1), fontsize=8, verticalalignment='center', horizontalalignment='center')
        ax.text(text_x, text_y, str(idx + 1), fontsize=8, verticalalignment='center',
                horizontalalignment='center', color='white', path_effects=[
                patheffects.withStroke(linewidth=2, foreground="black")])


    ax.set_title(f'Boustrophedon Path at {degree}-degree')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# The main function now correctly handles the angle normalization
def main_corrected(angle_degrees):
    # Sample polygon points
    points = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]
    polygon = Polygon(points)

    # Generate the Boustrophedon path with the corrected angle and line spacing
    path, num_segments = boustrophedon_path_corrected(polygon, line_spacing=0.9, angle_degrees=angle_degrees)
    print(f"Total line segments: {num_segments}")

    # Plot the path
    plot_path(polygon, path, angle_degrees)

# Run the main function with a corrected angle
main_corrected(47)  # You can change 130 to any angle you wish

