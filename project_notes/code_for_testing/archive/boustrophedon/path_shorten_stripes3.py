#!/usr/bin/env python
'''

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_shorten_stripes3.py
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

def write_segments_to_excel(segments, file_path, sheet_name):
    data = []
    for geom in segments:
        if isinstance(geom, LineString):
            data.append([geom.coords[0][0], geom.coords[0][1], geom.coords[-1][0], geom.coords[-1][1]])
        elif isinstance(geom, MultiLineString):
            for line in geom.geoms:
                data.append([line.coords[0][0], line.coords[0][1], line.coords[-1][0], line.coords[-1][1]])
        elif isinstance(geom, Point):
            data.append([geom.x, geom.y, geom.x, geom.y])  # Represent point as a zero-length line
    
    df = pd.DataFrame(data, columns=['Start_X', 'Start_Y', 'End_X', 'End_Y'])
    
    with pd.ExcelWriter(file_path, engine='openpyxl', mode='a') as writer:
        if sheet_name in writer.book.sheetnames:
            writer.book.remove(writer.book[sheet_name])
        df.to_excel(writer, sheet_name=sheet_name, index=False)

print("Starting script execution...")

folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = 'collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'
full_path = folder_path + file_path
polygon_sheet_name = 'RawInnerRings'
stripes_sheet_name = 'boustrphdn_trimmed'
output_sheet_name = 'stripes_trimmed'

# Read the inner ring
inner_ring = read_inner_ring(full_path, polygon_sheet_name)
print("Inner ring read successfully.")

# Read the line segments
line_segments = read_line_segments(full_path, stripes_sheet_name)
print(f"Line segments read successfully. Total segments: {len(line_segments)}")

# Create the circles
circle_data = [
    (Point(-10.52, -33.01), 6)
]
circles = [center.buffer(radius) for center, radius in circle_data]

# Clip the line segments
clipped_segments = [clip_line_at_circles(line, circles) for line in line_segments]
clipped_segments = [seg for seg in clipped_segments if seg is not None]
print(f"Line segments clipped. Remaining segments after clipping: {len(clipped_segments)}")

# Write the clipped segments to Excel
write_segments_to_excel(clipped_segments, full_path, output_sheet_name)
print(f"Clipped segments written to Excel sheet '{output_sheet_name}'.")

# Read the clipped segments from Excel for plotting
df_clipped = pd.read_excel(full_path, sheet_name=output_sheet_name)

print("Preparing to plot results...")

# Plotting
fig, ax = plt.subplots(figsize=(12, 12))

# Plot the inner ring
x, y = inner_ring.exterior.xy
ax.plot(x, y, color='green', linewidth=2, linestyle='dashed')

# Plot the circles
for center, radius in circle_data:
    circle_patch = Circle(center.coords[0], radius, fill=False, color='purple')
    ax.add_patch(circle_patch)

# Plot the clipped line segments
for _, row in df_clipped.iterrows():
    ax.plot([row['Start_X'], row['End_X']], [row['Start_Y'], row['End_Y']], color='blue', linewidth=2)

ax.set_aspect('equal')
ax.grid(True)
plt.title('Clipped Line Segments')
plt.xlabel('X')
plt.ylabel('Y')

print("Plot generated. Displaying results...")
plt.show()

print("Script execution completed.")