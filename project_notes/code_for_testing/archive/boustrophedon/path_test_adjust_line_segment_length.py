# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_adjust_line_segment_length.py
import numpy as np
import matplotlib.pyplot as plt
import csv
import pandas as pd

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

def adjust_line_length(start_point, end_point, length_to_adjust, adjust_start=False):
    """
    Adjusts the length of a line segment by a specified amount, either at the start or the end.

    :param start_point: Array of the starting point coordinates [x, y].
    :param end_point: Array of the ending point coordinates [x, y].
    :param length_to_adjust: The length by which the line should be adjusted.
    :param adjust_start: Boolean, if True adjusts the start point, otherwise adjusts the end point.
    :return: New start and end points of the line.
    """
    # Calculate the unit vector
    line_vector = end_point - start_point
    line_length = np.linalg.norm(line_vector)
    unit_vector = line_vector / line_length

    # Adjust the line length
    if adjust_start:
        # Adjusting the start point
        new_start_point = start_point + unit_vector * length_to_adjust
        #print("Adjusted start point:", new_start_point)
        return new_start_point, end_point
    else:
        # Adjusting the end point
        new_end_point = end_point - unit_vector * length_to_adjust
        #print("Adjusted end point:", new_end_point)
        return start_point, new_end_point

def plot_line(start_point, end_point, new_end_point=None):
    """
    Plots the original line and optionally a new end point.

    :param start_point: Array of the starting point coordinates [x, y].
    :param end_point: Array of the ending point coordinates [x, y].
    :param new_end_point: Optional array of the new ending point coordinates [x, y].
    """
    plt.figure(figsize=(8, 6))
    plt.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], label="Original Line")
    plt.scatter(*end_point, color='blue', label="Original End Point")
    plt.scatter(*start_point, color='green', label="Start Point")
    if new_end_point is not None:
        plt.scatter(*new_end_point, color='red', label="New End Point")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.title("Line with Adjusted Length")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def save_points_to_csv(points, file_name, starting_row_of_data):
    """
    Saves given points to a CSV file.

    :param points: List of points (each point is a numpy array or list with two elements).
    :param file_name: Name of the CSV file to save the points.
    :param starting_row_of_data: Starting value for the row reference.
    """
    with open(file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        for i, point in enumerate(points, start=starting_row_of_data):
            writer.writerow([i, point[0], point[1]])

# Update the main function for testing
def main():

	column_indexes = "L:M"  # Columns L and M (assuming columns start from A as 0, L would be 11 and M would be 12)

	rows_of_data = 4
	df = pd.read_excel(ods_file_path, engine='odf', sheet_name='Site_01_path_with_intercepts', 
	                   usecols=column_indexes, skiprows=(starting_row_of_data - 1), nrows=rows_of_data, header=None)  # Read the Excel file without treating any row as header
	df.columns = ['X', 'Y']  # Assign custom column names
	point1 = np.array(df.iloc[0])
	point2 = np.array(df.iloc[1])
	point3 = np.array(df.iloc[2])
	point4 = np.array(df.iloc[3])
	if DEBUG_MODE == 1:
		print("point1:", point1)
		print("point2:", point2)
		print("point3:", point3)
		print("point4:", point4)	

	_, new_end = adjust_line_length(point1, point2, length_to_shorten, adjust_start=False)
	new_start, _ = adjust_line_length(point3, point4, length_to_shorten, adjust_start=True)

#So we need a new ‘end’ for 2, 3 and a new start for 4, 5	
	if DEBUG_MODE == 1:
		print(f"New point {starting_row_of_data+1}:", new_end)
		plot_line(point1, point2, new_end_point=new_end)	 	# Plot the line with adjusted start point
		print(f"New point {starting_row_of_data+2}:", new_start)
		plot_line(point3, point4, new_end_point=new_start)	 	# Plot the line with adjusted start point
	
	# save the points to a .csv file

	points = [point1, new_end, new_start, point4]
	save_points_to_csv(points, uturn_points_csv, starting_row_of_data)

# Run the main function
length_to_shorten = -3.0  # using a negative number will extend the line (i.e. make it longer)
starting_row_of_data = 9
DEBUG_MODE = 1
uturn_points_csv = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/path_uturn_adjusted.csv'
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophedon_w_formulas_v1.ods'
main()
print(f"The next step is to open {uturn_points_csv} and copy the 2nd and 3rd lines into {ods_file_path} at row {starting_row_of_data + 1}")
print("After pasting and saving the data, run path_test_update_input_file.py")
print("Then run the path generator followed by path_test_plot_before_running_simulation.py to check the placement")
print(f"eoj: {script_name}")