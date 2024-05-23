#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .ods file that represent the bounds around an obstacle and calculates the center 
point and radius.  This will be used as input to additional path planning scripts.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_calc_circle_radius_and_center3.py

'''
print(f"Starting the script path_calc_circle_radius_and_center3.py...")
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from odf.opendocument import load
from odf.table import Table, TableRow, TableCell, TableColumn
from odf.text import P

def get_cell_text(cell):
    """Extract text from a cell."""
    paragraphs = cell.getElementsByType(P)
    text_content = "".join([p.firstChild.data for p in paragraphs if p.firstChild])
    return text_content

def read_data_from_ods(filename, sheet_name, reference_tag):
    doc = load(filename)
    sheet = None
    for s in doc.spreadsheet.getElementsByType(Table):
        if s.getAttribute("name") == sheet_name:
            sheet = s
            break
    
    if sheet is None:
        raise ValueError(f"Sheet {sheet_name} not found in {filename}")

    x_data = []
    y_data = []
    
    rows = sheet.getElementsByType(TableRow)
    header = [get_cell_text(cell) for cell in rows[0].getElementsByType(TableCell)]
    
    # Identify the column indices based on headers
    pose_x_idx = header.index('pose_x')
    pose_y_idx = header.index('pose_y')
    reference_idx = header.index('Reference 1')
    
    for row in rows[1:]:  # Skip the header row
        cells = row.getElementsByType(TableCell)
        if len(cells) > reference_idx:
            reference_value = get_cell_text(cells[reference_idx])
            if reference_value == reference_tag:
                x_value = float(get_cell_text(cells[pose_x_idx]))
                y_value = float(get_cell_text(cells[pose_y_idx]))
                x_data.append(x_value)
                y_data.append(y_value)
    
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
    formatted_center_x = round(center[0], 3)
    formatted_center_y = round(center[1], 3)
    formatted_radius = round(radius, 3)

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

def plot_data_and_circle(X, Y, xc, yc, R, additional_circle_center, additional_radius):
    circle = plt.Circle((xc, yc), R, color='blue', fill=False)
    additional_circle = plt.Circle(additional_circle_center, additional_radius, color='green', fill=False, linestyle='--')
    
    fig, ax = plt.subplots()
    ax.add_artist(circle)
    ax.add_artist(additional_circle)
    ax.scatter(X, Y, color='red')
    ax.set_aspect('equal', adjustable='datalim')
    plt.title('Circle Fit to Data with Additional Circle')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid()
    plt.show()

# File paths and sheet names
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
ods_filename = folder_path + 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
tag_reference = 'Obstacle 1'
data_sheet_name = 'SiteSurvey'
result_sheet_name = 'Obstacle'

# Read data from the ODS file
x_data, y_data = read_data_from_ods(ods_filename, data_sheet_name, tag_reference)

# Perform circle fitting
xc, yc, R = circle_fit(x_data, y_data)

# Save the center and radius data to the ODS file
save_to_ods(ods_filename, result_sheet_name, tag_reference, (xc, yc), R)
print(f'Data saved to sheet {result_sheet_name} in {ods_filename}')
print(f"Circle Center: ({xc}, {yc}), Radius: {R}")

# Plot the data and the circle
plot_data_and_circle(x_data, y_data, xc, yc, R, additional_circle_center=(17.3, -9.1), additional_radius=2.4)
