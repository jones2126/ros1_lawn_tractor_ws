#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and modify the path to avoid the obstacle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_test_obstacle1.py

'''
import pandas as pd

# Path to the .ods file
file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.ods'


# Function to read the "Obstacle 1" sheet
def read_obstacle_1(file_path):
	sheet_to_read = 'Obstacle 1'
	try:
		data = pd.read_excel(file_path, sheet_name=sheet_to_read, engine='odf')
		print(f"Contents of {sheet_to_read} sheet:")  
		print(data)
	except Exception as e:
		print(f"An error occurred: {e}")

# Read and print the "Obstacle 1" sheet
read_obstacle_1(file_path)



# import ezodf
# import pandas as pd

# # Path to the .ods file
# file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.ods'

# # Load the .ods file
# spreadsheet = ezodf.opendoc(file_path)

# # Access the "Obstacle 1" sheet
# sheet = spreadsheet.sheets['Obstacle 1']

# # Read the sheet data into a pandas DataFrame
# data = []
# for row in sheet.rows():
#     data.append([cell.value for cell in row])

# df = pd.DataFrame(data)
# print("Contents of 'Obstacle 1' sheet:")
# print(df)
