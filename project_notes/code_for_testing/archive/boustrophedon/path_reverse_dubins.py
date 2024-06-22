#!/usr/bin/env python

'''
Read the x, y data for the path and flip (i.e. reverse) the sequence.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_reverse_dubins.py
'''
print("Starting the reverse process")
import pandas as pd

# Define the input file path
input_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'

# Load the Excel file
excel_data = pd.ExcelFile(input_file_path)

# Load the 'path_dubins' sheet into a DataFrame with a one-row header
path_dubins_df = pd.read_excel(input_file_path, sheet_name='path_dubins', header=0)

# Reverse the order of the DataFrame
reversed_path_dubins_df = path_dubins_df.iloc[::-1].reset_index(drop=True)

# Define the output file path (same as input, we just add a new sheet)
output_file_path = input_file_path

# Save the reversed DataFrame into a new sheet in the same Excel file
with pd.ExcelWriter(output_file_path, engine='openpyxl', mode='a') as writer:
    reversed_path_dubins_df.to_excel(writer, sheet_name='path_dubins_reversed', index=False, header=True)

print(f"Reversed path saved to sheet 'path_dubins_reversed' in the file: {output_file_path}")

