# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage_8_degree.py
import pandas as pd
import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate
import matplotlib.pyplot as plt

# Define the boustrophedon path function for 8-degree lines
def boustrophedon_path_8_degree_complete(polygon, line_spacing=0.9):
    # Rotate the polygon to align the slicing lines with the y-axis
    #rotated_polygon = rotate(polygon, -8, origin='centroid', use_radians=False)
    rotated_polygon = rotate(polygon, -19, origin='centroid', use_radians=False)
    minx, miny, maxx, maxy = rotated_polygon.bounds

    # Calculate the length needed for lines to cover the entire area after rotation
    #diag_length = max(maxx - minx, maxy - miny) / np.cos(np.radians(8))
    diag_length = max(maxx - minx, maxy - miny) / np.cos(np.radians(19))

    # Determine the extended bounds for the lines
    extended_bounds = np.ceil(diag_length / line_spacing)
    extended_minx = minx - extended_bounds
    extended_maxx = maxx + extended_bounds
    extended_miny = miny - extended_bounds
    extended_maxy = maxy + extended_bounds

    # Calculate the number of lines needed, considering the line spacing
    num_lines = int(np.ceil((extended_maxx - extended_minx) / line_spacing))

    path = []
    # Generate lines from the extended bottom-left to the extended top-right of the bounding box
    for i in range(num_lines + 1):
        # Calculate the current x-coordinate for the line
        x = extended_minx + line_spacing * i
        # Create a vertical line that will be rotated to 8 degrees
        line = LineString([(x, extended_miny), (x, extended_maxy)])
        # Rotate the line to an 8-degree angle
        #rotated_line = rotate(line, 8, origin='centroid', use_radians=False)
        rotated_line = rotate(line, 19, origin='centroid', use_radians=False)
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

# Define the plot function
def plot_path(polygon, path_lines):
    fig, ax = plt.subplots()
    x, y = polygon.exterior.xy
    ax.plot(x, y, color='blue', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)

    for line in path_lines:
        x, y = line.xy
        ax.plot(x, y, color='red', linewidth=2, alpha=0.7, zorder=2)

    #ax.set_title('Boustrophedon Path at 8-degree')
    ax.set_title('Boustrophedon Path at 19-degree')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

# Define the main function to run the program
def main():
    # Sample polygon points
    hard_coded_points = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]

    csv_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'  # Path to the CSV file
    polygon_data = pd.read_csv(csv_path)
    polygon_points_from_csv = list(zip(polygon_data['X'], polygon_data['Y']))

    polygon = Polygon(polygon_points_from_csv)
    #polygon = Polygon(hard_coded_points)

    # Generate the Boustrophedon path with 8-degree angle lines and 0.9m separation
    path, num_segments = boustrophedon_path_8_degree_complete(polygon, line_spacing=0.9)
    print(f"Total line segments: {num_segments}")

    # Plot the path
    plot_path(polygon, path)

# Run the main function
main()
