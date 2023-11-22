# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_boustrophedon_coverage_minus_10_degree_real_data.py
import numpy as np
import pandas as pd
from shapely.geometry import Polygon, LineString, MultiLineString
from shapely.affinity import rotate
import matplotlib.pyplot as plt

# Function to create a boustrophedon path at a negative angle
def boustrophedon_path_negative_10_degree(polygon, line_spacing=0.9):
    rotated_polygon = rotate(polygon, 10, origin='centroid', use_radians=False)
    minx, miny, maxx, maxy = rotated_polygon.bounds
    diag_length = max(maxx - minx, maxy - miny) / np.cos(np.radians(10))
    extended_bounds = np.ceil(diag_length / line_spacing)
    extended_minx = minx - extended_bounds
    extended_maxx = maxx + extended_bounds
    extended_miny = miny - extended_bounds
    extended_maxy = maxy + extended_bounds
    num_lines = int(np.ceil((extended_maxx - extended_minx) / line_spacing))
    path = []
    for i in range(num_lines + 1):
        x = extended_minx + line_spacing * i
        line = LineString([(x, extended_miny), (x, extended_maxy)])
        rotated_line = rotate(line, -10, origin='centroid', use_radians=False)
        intersection = polygon.intersection(rotated_line)
        if not intersection.is_empty:
            if isinstance(intersection, LineString):
                path.append(intersection)
            elif isinstance(intersection, MultiLineString):
                # for single_line in intersection:
                #     path.append(single_line)

                for single_line in intersection.geoms:
                    path.append(single_line)

    return path, len(path)

# Function to plot the boustrophedon path and return the figure and axes
def plot_path_from_csv(polygon, path_lines, degree):
    fig, ax = plt.subplots()
    x, y = polygon.exterior.xy
    ax.plot(x, y, color='blue', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
    for line in path_lines:
        x, y = line.xy
        ax.plot(x, y, color='red', linewidth=2, alpha=0.7, zorder=2)
    ax.set_title(f'Boustrophedon Path at {degree}-degree using CSV data')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.grid(True)
    plt.axis('equal')
    return fig, ax

# Main function to run the script
def main():
    csv_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'  # Path to the CSV file
    polygon_data = pd.read_csv(csv_path)
    polygon_points_from_csv = list(zip(polygon_data['X'], polygon_data['Y']))
    polygon = Polygon(polygon_points_from_csv)

    hard_coded_points = [(5, 0), (6, 4), (7, 5), (8, 10), (7, 11), (5, 10), (3, 11), (1, 10), (0, 5), (1, 3)]
    #polygon = Polygon(hard_coded_points)

    path, num_segments = boustrophedon_path_negative_10_degree(polygon, line_spacing=0.9)
    fig, ax = plot_path_from_csv(polygon, path, degree=-10)
    plt.show()  # If you want to display the plot interactively, otherwise remove this line for batch runs

if __name__ == '__main__':
    main()
