
import pandas as pd
from shapely.geometry import Polygon, LineString, MultiLineString
import numpy as np
import matplotlib.pyplot as plt

def create_polygon(dataframe):
    """
    Create a shapely polygon from the given dataframe with 'X' and 'Y' columns.
    """
    points = dataframe[['X', 'Y']].values
    polygon = Polygon(points)
    return polygon


def boustrophedon_path(polygon, line_spacing=1.0):
    """
    Create a Boustrophedon path for a given polygon.

    Parameters:
    - polygon: Shapely Polygon object
    - line_spacing: distance between the parallel lines in the Boustrophedon pattern

    Returns:
    - path: a list of Shapely LineString objects representing the Boustrophedon path
    """
    minx, miny, maxx, maxy = polygon.bounds
    y = miny
    path = []
    while y < maxy:
        # Create a line at the current y-coordinate
        line = LineString([(minx, y), (maxx, y)])
        # Find the intersection of the line with the polygon
        intersection = polygon.intersection(line)
        if intersection.is_empty:
            # If there is no intersection, continue to the next line
            y += line_spacing
            continue
        if isinstance(intersection, LineString):
            # If the intersection is a line, add it to the path
            path.append(intersection)
        elif isinstance(intersection, MultiLineString):
            # If the intersection is multiple lines, add each one to the path
            for line in intersection:
                path.append(line)
        # Move up to the next line
        y += line_spacing
    return path


def plot_path(polygon, path):
    """
    Plot the Boustrophedon path within the given polygon.
    
    Parameters:
    - polygon: Shapely Polygon object
    - path: a list of Shapely LineString objects representing the Boustrophedon path
    """
    x,y = polygon.exterior.xy
    plt.figure()
    plt.plot(x, y, 'b-', linewidth=2)  # Plot the polygon
    for line in path:
        x,y = line.xy
        plt.plot(x, y, 'r-', linewidth=1)  # Plot the path
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Boustrophedon Path')
    plt.axis('equal')
    plt.show()


def main(file_path):
    # Read the CSV file to get the polygon points
    data = pd.read_csv(file_path)
    # Create the polygon from the data
    polygon = create_polygon(data)
    # Generate the Boustrophedon path
    boustrophedon_lines = boustrophedon_path(polygon, line_spacing=1.0)
    # Plot the path
    plot_path(polygon, boustrophedon_lines)


# The following code will be outside the main function in the script
# main('/path/to/your/csv/file.csv') # Uncomment this line in the script to run the main function
