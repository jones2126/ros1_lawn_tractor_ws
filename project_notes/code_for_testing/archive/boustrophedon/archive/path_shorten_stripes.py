#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .xlsx file and outputs a series of concentric rings and stores them in a new sheet in the .xlsx file.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_shorten_stripes.py
'''
import pandas as pd
from shapely.geometry import Polygon, Point, LineString
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

def clip_line_at_circle(line, circle):
# '''
# This function checks if a line intersects with the circle and, if it does, returns the 
# intersection. The line.intersection(circle) call is what actually computes the clipping points.
# '''    
    if not line.intersects(circle):
        return None
    return line.intersection(circle)

folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = 'collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
full_path = folder_path + file_path

polygon_sheet_name = 'RawInnerRings'
stripes_sheet_name = 'boustrphdn_trimmed'

# Read the inner ring
inner_ring = read_inner_ring(full_path, polygon_sheet_name)

# Read the line segments
line_segments = read_line_segments(full_path, stripes_sheet_name)

# Create the circle
circle_center = Point(-20, -10)
#circle_center = Point(-10.52, -33.01)
circle_radius = .5
circle = circle_center.buffer(circle_radius)

# Clip the line segments
clipped_segments = [clip_line_at_circle(line, circle) for line in line_segments]
clipped_segments = [seg for seg in clipped_segments if seg is not None]

# Plotting
fig, ax = plt.subplots(figsize=(12, 12))

# Plot the inner ring
x, y = inner_ring.exterior.xy
ax.plot(x, y, color='green', linewidth=2, linestyle='dashed')

# Plot the circle
circle_patch = Circle(circle_center.coords[0], circle_radius, fill=False, color='purple')
ax.add_patch(circle_patch)

# Plot the original line segments
for line in line_segments:
    x, y = line.xy
    ax.plot(x, y, color='red', linewidth=1)

# Plot the clipped line segments
for line in clipped_segments:
    if isinstance(line, LineString):
        x, y = line.xy
        ax.plot(x, y, color='blue', linewidth=2)
    elif isinstance(line, Point):
        ax.plot(line.x, line.y, 'bo', markersize=5)

ax.set_aspect('equal')
ax.grid(True)
plt.title('Line Segments Clipped at Circle Intersection')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

# Print the clipped segments coordinates
for i, seg in enumerate(clipped_segments):
    print(f"Clipped segment {i+1}:")
    if isinstance(seg, LineString):
        print(f"Start: ({seg.coords[0][0]:.2f}, {seg.coords[0][1]:.2f})")
        print(f"End: ({seg.coords[-1][0]:.2f}, {seg.coords[-1][1]:.2f})")
    elif isinstance(seg, Point):
        print(f"Point: ({seg.x:.2f}, {seg.y:.2f})")
    print()