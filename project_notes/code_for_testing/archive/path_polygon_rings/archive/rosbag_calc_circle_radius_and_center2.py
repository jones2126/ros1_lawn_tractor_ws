#!/usr/bin/env python

'''
Script that reads the pose_x and pose_y data from a .csv file and outputs a series of concentric rings.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/rosbag_calc_circle_radius_and_center2.py

'''
print(f"Starting the process...")
import rosbag
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import pandas as pd
from odf.opendocument import load
from odf.table import Table, TableColumn, TableRow, TableCell
from odf.text import P

def save_to_ods(filename, sheet_name, x_data, y_data, center, radius):
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

    # Add header row for X and Y data
    header_row = TableRow()
    header_x = TableCell()
    header_x.addElement(P(text="X"))
    header_y = TableCell()
    header_y.addElement(P(text="Y"))
    header_row.addElement(header_x)
    header_row.addElement(header_y)
    table.addElement(header_row)

    # Add data rows
    for x, y in zip(x_data, y_data):
        row = TableRow()
        cell_x = TableCell()
        cell_x.addElement(P(text=str(x)))
        cell_y = TableCell()
        cell_y.addElement(P(text=str(y)))
        row.addElement(cell_x)
        row.addElement(cell_y)
        table.addElement(row)

    # Add a blank row to separate the circle data
    blank_row = TableRow()
    table.addElement(blank_row)

    # Add labels and data for circle center and radius
    center_x_label_row = TableRow()
    center_x_label = TableCell()
    center_x_label.addElement(P(text="Circle Center X"))
    center_x_value = TableCell()
    center_x_value.addElement(P(text=str(center[0])))
    center_x_label_row.addElement(center_x_label)
    center_x_label_row.addElement(center_x_value)
    table.addElement(center_x_label_row)

    center_y_label_row = TableRow()
    center_y_label = TableCell()
    center_y_label.addElement(P(text="Circle Center Y"))
    center_y_value = TableCell()
    center_y_value.addElement(P(text=str(center[1])))
    center_y_label_row.addElement(center_y_label)
    center_y_label_row.addElement(center_y_value)
    table.addElement(center_y_label_row)

    radius_label_row = TableRow()
    radius_label = TableCell()
    radius_label.addElement(P(text="Circle Radius"))
    radius_value = TableCell()
    radius_value.addElement(P(text=str(radius)))
    radius_label_row.addElement(radius_label)
    radius_label_row.addElement(radius_value)
    table.addElement(radius_label_row)

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

# Replace with your ROS bag file path
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = '62Collins_perimiter_1_2024-04-17-18-34-40.bag'
start_time = 300  # Start time in seconds
end_time = 368   # End time in seconds
odom_topic = '/odom'
ods_filename = folder_path + 'collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
sheet_name = 'Obstacle1'

bag = rosbag.Bag(folder_path + file_path)
x_data = []
y_data = []
count = 0 
first_timestamp = None

for topic, msg, t in bag.read_messages(topics=[odom_topic]):
    timestamp_seconds = t.to_sec()

    if first_timestamp is None:
        first_timestamp = timestamp_seconds

    relative_time = timestamp_seconds - first_timestamp
    if start_time <= relative_time <= end_time:
        if topic == '/odom':
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            x_data.append(x)
            y_data.append(y)
            count += 1

bag.close()

# Perform circle fitting
xc, yc, R = circle_fit(x_data, y_data)

# Save data to the ODS file
save_to_ods(ods_filename, sheet_name, x_data, y_data, (xc, yc), R)
print(f'Data saved to sheet {sheet_name} in {ods_filename}')
print(f"count: {count}")
print(f"Circle Center: ({xc}, {yc}), Radius: {R}")

# Read the CSV file
df = pd.DataFrame({'X': x_data, 'Y': y_data})

plot_data_and_circle(df['X'].values, df['Y'].values, xc, yc, R, additional_circle_center = (17.3, -9.1), additional_radius = 2.4)
