# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_calc_circle_radius_and_center.py

import rosbag
import csv
import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
import pandas as pd

def save_to_csv(filename, x_data, y_data):
    with open(filename, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['X', 'Y'])
        for x, y in zip(x_data, y_data):
            csvwriter.writerow([x, y])

# Replace with your ROS bag file path
folder_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/'
file_path = '62Collins_perimiter_1_2024-04-17-18-34-40.bag'
start_time = 300  # Start time in seconds
end_time = 368   # End time in seconds
odom_topic = '/odom'

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
            count = count + 1

bag.close()

csv_filename = folder_path + '2024-05-14-extracted_data.csv'
save_to_csv(csv_filename, x_data, y_data)
print(f'Data saved to {csv_filename}')
print(f"count: {count}")

# Function to fit a circle to X, Y data and return the center and radius
def circle_fit(X, Y):
    # Initial guess for the circle's center is the mean of the points
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
    # The original circle
    circle = plt.Circle((xc, yc), R, color='blue', fill=False)
    # The additional circle
    
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

# Read the CSV file
df = pd.read_csv(csv_filename)

# Perform circle fitting
xc, yc, R = circle_fit(df['X'].values, df['Y'].values)

# Print out the center and radius
print(f"Circle Center: ({xc}, {yc}), Radius: {R}")

plot_data_and_circle(df['X'].values, df['Y'].values, xc, yc, R, additional_circle_center = (17.3, -9.1), additional_radius = 2.4)