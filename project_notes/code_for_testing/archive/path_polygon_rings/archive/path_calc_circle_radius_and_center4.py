#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .ods file that represent the bounds around an obstacle and calculates the center 
point and radius.  This will be used as input to additional path planning scripts.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_calc_circle_radius_and_center4.py

'''
print(f"Starting the script path_calc_circle_radius_and_center4.py...")
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from odf.opendocument import load
from odf.table import Table, TableRow, TableCell, TableColumn
from odf.text import P

def read_data_from_ods(filename, sheet_name, reference_tag):
    # Read data using pandas
    data = pd.read_excel(filename, engine='odf', sheet_name=sheet_name)
    print("Columns in SiteSurvey sheet:", data.columns)

    # Adjust these column names based on the actual names in your ODS file
    reference_col = 'Reference 1'
    pose_x_col = 'pose_x'
    pose_y_col = 'pose_y'

    # Filter data based on the reference_tag
    filtered_data = data[data[reference_col] == reference_tag]
    
    x_data = filtered_data[pose_x_col].tolist()
    y_data = filtered_data[pose_y_col].tolist()
    
    return x_data, y_data

def save_to_ods(filename, sheet_name, reference_tag, center, radius):
    doc = load(filename)
    sheets = doc.getElementsByType(Table)
    
    # Remove the sheet if it already exists
    for sheet in sheets:
        if sheet.getAttribute("name") == sheet_name:
            doc.spreadsheet.removeChild(sheet)
            break

    # Create a new sheet
    table = Table(name=sheet_name)

    # Add columns
    table.addElement(TableColumn())
    table.addElement(TableColumn())
    table.addElement(TableColumn())
    table.addElement(TableColumn())

    # Add header row
    header_row = TableRow()
    header_reference = TableCell()
    header_reference.addElement(P(text="Reference"))
    header_x = TableCell()
    header_x.addElement(P(text="X"))
    header_y = TableCell()
    header_y.addElement(P(text="Y"))
    header_radius = TableCell()
    header_radius.addElement(P(text="Radius"))
    header_row.addElement(header_reference)
    header_row.addElement(header_x)
    header_row.addElement(header_y)
    header_row.addElement(header_radius)
    table.addElement(header_row)

    # Format values to 2 decimal places
    formatted_center_x = round(center[0], 2)
    formatted_center_y = round(center[1], 2)
    formatted_radius = round(radius, 2)

    print(f"Saving values: Reference={reference_tag}, X={formatted_center_x}, Y={formatted_center_y}, Radius={formatted_radius}")

    # Add data row
    data_row = TableRow()
    reference_cell = TableCell(valuetype="string")
    reference_cell.addElement(P(text=reference_tag))
    x_cell = TableCell(valuetype="float", value=formatted_center_x)
    y_cell = TableCell(valuetype="float", value=formatted_center_y)
    radius_cell = TableCell(valuetype="float", value=formatted_radius)
    data_row.addElement(reference_cell)
    data_row.addElement(x_cell)
    data_row.addElement(y_cell)
    data_row.addElement(radius_cell)
    table.addElement(data_row)

    # Append the new table to the spreadsheet
    doc.spreadsheet.addElement(table)

    # Save the document
    doc.save(filename)

# def read_obstacle_data_from_ods(filename, sheet_name):
#     # Read data using pandas
#     data = pd.read_excel(filename, engine='odf', sheet_name=sheet_name)
#     print("Columns in Obstacle sheet:", data.columns)

#     # Adjust these column names based on the actual names in your ODS file
#     reference_col = 'Reference'
#     x_col = 'X'
#     y_col = 'Y'
#     radius_col = 'Radius'
    
#     # Ensure columns exist
#     if reference_col not in data.columns or x_col not in data.columns or y_col not in data.columns or radius_col not in data.columns:
#         print("Error: Expected columns not found in the Obstacle sheet.")
#         print("Actual columns:", data.columns)
#         return []

#     reference = data[reference_col].values
#     x = data[x_col].values
#     y = data[y_col].values
#     radius = data[radius_col].values

#     obstacle_data = list(zip(reference, x, y, radius))
    
#     return obstacle_data

def read_obstacle_data_from_ods(filename, sheet_name):
    # Read data using pandas
    data = pd.read_excel(filename, engine='odf', sheet_name=sheet_name)
    print("Columns in Obstacle sheet:", data.columns)
    
    # Rename the columns for clarity
    data.columns = ['Reference', 'X', 'Y', 'Radius']

    # Extract the data into separate variables
    reference = data['Reference'].values
    x = data['X'].values
    y = data['Y'].values
    radius = data['Radius'].values

    # Print the extracted variables for debugging
    print("Extracted References:", reference)
    print("Extracted X coordinates:", x)
    print("Extracted Y coordinates:", y)
    print("Extracted Radii:", radius)

    # Combine the data into a list of tuples
    obstacle_data = list(zip(reference, x, y, radius))
    
    return obstacle_data


def calculate_circle_arc(circle_center, radius, num_points):
    segments = []
    angles = np.linspace(0, 2 * np.pi, num_points + 1)
    for i in range(num_points):
        start_angle = angles[i]
        end_angle = angles[i + 1]
        start_point = (circle_center[0] + radius * np.cos(start_angle), circle_center[1] + radius * np.sin(start_angle))
        end_point = (circle_center[0] + radius * np.cos(end_angle), circle_center[1] + radius * np.sin(end_angle))
        segments.append((start_point, end_point))
    return segments

def save_segments_to_ods(filename, reference_tag, segments):
    doc = load(filename)
    sheet_name = reference_tag
    
    # Remove the sheet if it already exists
    sheets = doc.getElementsByType(Table)
    for sheet in sheets:
        if sheet.getAttribute("name") == sheet_name:
            doc.spreadsheet.removeChild(sheet)
            break

    # Create a new sheet
    table = Table(name=sheet_name)

    # Add columns
    table.addElement(TableColumn())
    table.addElement(TableColumn())
    table.addElement(TableColumn())
    table.addElement(TableColumn())

    # Add header row
    header_row = TableRow()
    header_start_x = TableCell()
    header_start_x.addElement(P(text="Start X"))
    header_start_y = TableCell()
    header_start_y.addElement(P(text="Start Y"))
    header_end_x = TableCell()
    header_end_x.addElement(P(text="End X"))
    header_end_y = TableCell()
    header_end_y.addElement(P(text="End Y"))
    header_row.addElement(header_start_x)
    header_row.addElement(header_start_y)
    header_row.addElement(header_end_x)
    header_row.addElement(header_end_y)
    table.addElement(header_row)

    # Add segment rows
    for start_point, end_point in segments:
        segment_row = TableRow()
        start_x_cell = TableCell(valuetype="float", value=round(start_point[0], 2))
        start_y_cell = TableCell(valuetype="float", value=round(start_point[1], 2))
        end_x_cell = TableCell(valuetype="float", value=round(end_point[0], 2))
        end_y_cell = TableCell(valuetype="float", value=round(end_point[1], 2))
        segment_row.addElement(start_x_cell)
        segment_row.addElement(start_y_cell)
        segment_row.addElement(end_x_cell)
        segment_row.addElement(end_y_cell)
        table.addElement(segment_row)

    # Append the new table to the spreadsheet
    doc.spreadsheet.addElement(table)

    # Save the document
    doc.save(filename)

# Function to fit a circle to X, Y data and return the center and radius
def circle_fit(X, Y):
    x_m = np.mean(X)
    y_m = np.mean(Y)

    def calc_R(xc, yc):
        return np.sqrt((X - xc)**2 + (Y - yc)**2)

    def f_2(c):
        Ri = calc_R(*c)
        return Ri - Ri.mean()

    center_estimate = x_m, y_m
    center_ier = least_squares(f_2, center_estimate)
    xc, yc = center_ier.x

    Ri = calc_R(xc, yc)
    R = Ri.mean()  # The mean of the distances to the center is the radius

    return xc, yc, R

def plot_data_and_circle(X, Y, xc, yc, R, segments=None):
    circle = plt.Circle((xc, yc), R, color='blue', fill=False)
    
    fig, ax = plt.subplots()
    ax.add_artist(circle)
    ax.scatter(X, Y, color='red')
    
    if segments:
        for start_point, end_point in segments:
            line = plt.Line2D((start_point[0], end_point[0]), (start_point[1], end_point[1]), color='green')
            ax.add_line(line)
    
    ax.set_aspect('equal', adjustable='datalim')
    plt.title('Circle Fit to Data with Line Segments')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid()
    plt.show()

# File paths and sheet names
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
ods_filename = folder_path + 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
data_sheet_name = 'SiteSurvey'
tag_reference = 'Obstacle 1'
result_sheet_name = 'Obstacle'

# Read data from the ODS file
x_data, y_data = read_data_from_ods(ods_filename, data_sheet_name, tag_reference)

# Perform circle fitting
xc, yc, R = circle_fit(x_data, y_data)

# Save the center and radius data to the ODS file
save_to_ods(ods_filename, result_sheet_name, tag_reference, (xc, yc), R)
print(f'Data saved to sheet {result_sheet_name} in {ods_filename}')
print(f"Circle Center: ({xc}, {yc}), Radius: {R}")

# Read obstacle data from the 'Obstacle' sheet
obstacle_data = read_obstacle_data_from_ods(ods_filename, result_sheet_name)
print(f"Obstacle data: {obstacle_data}")

# Calculate and save line segments for each obstacle
num_points = 20
all_segments = []
for reference, x, y, radius in obstacle_data:
    print("in loop to build segments")
    print(reference, x, y)
    segments = calculate_circle_arc((x, y), radius, num_points)
    save_segments_to_ods(ods_filename, reference, segments)
    all_segments.extend(segments)

# Plot the data, the circle, and the line segments
plot_data_and_circle(x_data, y_data, xc, yc, R, segments=all_segments)
