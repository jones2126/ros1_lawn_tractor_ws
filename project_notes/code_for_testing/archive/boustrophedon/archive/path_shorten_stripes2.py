#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .xlsx file and outputs a series of concentric rings and stores them in a new sheet in the .xlsx file.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/archive/path_shorten_stripes2.py
'''
import pandas as pd
from shapely.geometry import Polygon, Point, LineString, MultiLineString
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def read_inner_ring(xlsx_file_path, sheet_name):
    df = pd.read_excel(xlsx_file_path, sheet_name)
    filtered_df = df[df['Path_Index'] == 0]
    polygon_points = list(zip(filtered_df['X'], filtered_df['Y']))
    return Polygon(polygon_points)

def read_line_segments(xlsx_file_path, sheet_name):
    df = pd.read_excel(xlsx_file_path, sheet_name, header=None)
    return [LineString([(row[0], row[1]), (row[2], row[3])]) for _, row in df.iterrows()]

def clip_line_at_circles(line, circles):
    result = line
    for circle in circles:
        if result.intersects(circle):
            result = result.difference(circle)
    return result if not result.is_empty else None

folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = 'collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
full_path = folder_path + file_path

polygon_sheet_name = 'RawInnerRings'
stripes_sheet_name = 'boustrphdn_trimmed'

# Read the inner ring
inner_ring = read_inner_ring(full_path, polygon_sheet_name)

# Read the line segments
line_segments = read_line_segments(full_path, stripes_sheet_name)

# Create the circles
circle_data = [
    (Point(-20, -10), 0.5),
    (Point(-10.52, -33.01), 6),
    (Point(-21.5, -9), 0.5)
]
circles = [center.buffer(radius) for center, radius in circle_data]

# Clip the line segments
clipped_segments = [clip_line_at_circles(line, circles) for line in line_segments]
clipped_segments = [seg for seg in clipped_segments if seg is not None]

# Plotting
fig, ax = plt.subplots(figsize=(12, 12))

# Plot the inner ring
x, y = inner_ring.exterior.xy
ax.plot(x, y, color='green', linewidth=2, linestyle='dashed')

# Plot the circles
for center, radius in circle_data:
    circle_patch = Circle(center.coords[0], radius, fill=False, color='purple')
    ax.add_patch(circle_patch)

# Plot the original line segments
for line in line_segments:
    x, y = line.xy
    ax.plot(x, y, color='red', linewidth=1)

# Plot the clipped line segments
for geom in clipped_segments:
    if isinstance(geom, LineString):
        x, y = geom.xy
        ax.plot(x, y, color='blue', linewidth=2)
    elif isinstance(geom, MultiLineString):
        for line in geom.geoms:
            x, y = line.xy
            ax.plot(x, y, color='blue', linewidth=2)
    elif isinstance(geom, Point):
        ax.plot(geom.x, geom.y, 'bo', markersize=5)

ax.set_aspect('equal')
ax.grid(True)
plt.title('Line Segments Clipped at Circle Intersections')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

# Print the clipped segments coordinates
for i, geom in enumerate(clipped_segments):
    print(f"Clipped segment {i+1}:")
    if isinstance(geom, LineString):
        print(f"  Start: ({geom.coords[0][0]:.2f}, {geom.coords[0][1]:.2f})")
        print(f"  End: ({geom.coords[-1][0]:.2f}, {geom.coords[-1][1]:.2f})")
    elif isinstance(geom, MultiLineString):
        for j, line in enumerate(geom.geoms):
            print(f"  Part {j+1}:")
            print(f"    Start: ({line.coords[0][0]:.2f}, {line.coords[0][1]:.2f})")
            print(f"    End: ({line.coords[-1][0]:.2f}, {line.coords[-1][1]:.2f})")
    elif isinstance(geom, Point):
        print(f"Point: ({geom.x:.2f}, {geom.y:.2f})")
    print()