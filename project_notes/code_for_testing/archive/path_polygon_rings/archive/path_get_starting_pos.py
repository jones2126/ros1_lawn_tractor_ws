#!/usr/bin/env python

'''
Script to find intersections between a robot's path and a circular obstacle, and modify the path to avoid the obstacle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_get_starting_pos.py

'''
import pandas as pd
import numpy as np

# Function to get ring starting positions
def get_ring_starting_positions(ods_file_path):
    df = pd.read_excel(ods_file_path, sheet_name='RawInnerRings', engine='odf')
    path_index_changes = df[df['Path_Index'].diff() != 0].reset_index(drop=True)
    ring_starting_positions = [(df.iloc[0]['X'], df.iloc[0]['Y'])] + list(zip(path_index_changes['X'], path_index_changes['Y']))
    
    # Remove duplicates if the first position is repeated
    if ring_starting_positions[0] == ring_starting_positions[1]:
        ring_starting_positions.pop(1)
    
    return ring_starting_positions

# Function to update Path_Index based on starting positions
def update_path_index_based_on_starting_positions(ods_file_path, ring_starting_positions):
    df = pd.read_excel(ods_file_path, sheet_name='NewRingPath', engine='odf')
    path_index = 0

    for i in range(len(df)):
        x = df.at[i, 'X']
        y = df.at[i, 'Y']

        # Check if the current position matches any of the starting positions
        for j, (start_x, start_y) in enumerate(ring_starting_positions):
            if np.isclose(x, start_x) and np.isclose(y, start_y):
                path_index = j
                break
        
        df.at[i, 'Path_Index'] = path_index
    
    return df

# Example usage
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.ods'
print("starting")
print("getting starting positions")
ring_starting_positions = get_ring_starting_positions(ods_file_path)
print("Starting positions:", ring_starting_positions)
print("updating Path_Index")
updated_new_ring_path = update_path_index_based_on_starting_positions(ods_file_path, ring_starting_positions)

# Read existing sheets
with pd.ExcelFile(ods_file_path, engine='odf') as xls:
    sheet_names = xls.sheet_names
    sheet_data = {sheet: pd.read_excel(xls, sheet_name=sheet) for sheet in sheet_names}

# Update the NewRingPath sheet data
sheet_data['NewRingPath'] = updated_new_ring_path

# Write all sheets back to the .ods file
with pd.ExcelWriter(ods_file_path, engine='odf') as writer:
    for sheet_name, df in sheet_data.items():
        df.to_excel(writer, sheet_name=sheet_name, index=False)

print("Path_Index updated and saved to 'NewRingPath' sheet.")
