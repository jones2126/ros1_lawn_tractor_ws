#!/usr/bin/env python

'''

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/build_concentric_polygon_path3.py
'''
print("Running build_concentric_polygon_path3.py")


import pandas as pd

def process_arc_path(file_path):
    # Read the specific sheet from the Excel file
    df = pd.read_excel(file_path, sheet_name='ArcPath')

    # Extract the unique values in the 'Path_Index' column
    unique_path_indices = df['Path_Index'].unique().tolist()

    # Reverse the order of the list
    unique_path_indices.reverse()

    # Store Arc_X and Arc_Y values in lists for each Path_Index
    paths = {}
    for path_index in unique_path_indices:
        path_data = df[df['Path_Index'] == path_index]
        arc_x_values = path_data['Arc_X'].tolist()
        arc_y_values = path_data['Arc_Y'].tolist()
        paths[path_index] = {'Arc_X': arc_x_values, 'Arc_Y': arc_y_values}

    return unique_path_indices, paths

def process_intersection_points(file_path):
    # Read the specific sheet from the Excel file
    df = pd.read_excel(file_path, sheet_name='IntersectionPoints')

    # Extract the values in the 'Row_Index' column
    row_index_values = df['Row_Index'].tolist()

    # Create the 'start' and 'end' lists
    start = row_index_values[::2]  # Start with the first item and then every other one
    end = row_index_values[1::2]   # Start with the second item and then every other one

    # Reverse the lists.  I'm doing this because I need to use these references to inject the obstacle avoidance path and 
    # I need to do that from the bottom of the list and work up so the references remain valid.
    start.reverse()
    end.reverse()

    return start, end

# Define the path to the Excel file
xlsx_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/site1_20240513/collins_dr_62_A_from_rosbag_step1_20240513_2.xlsx'

# Call the functions and store the returned values
start, end = process_intersection_points(xlsx_file_path)
unique_path_indices, paths = process_arc_path(xlsx_file_path)

# Print the lists returned from process_intersection_points
print("Start list:", start)
print("End list:", end)

# Print the unique Path_Index list and the Arc_X and Arc_Y values for each Path_Index
print("Unique Path_Index list (reversed):", unique_path_indices)

# Read the specific sheet from the Excel file
df = pd.read_excel(xlsx_file_path, sheet_name='RawInnerRings')

# Print the length of the RawInnerRings DataFrame
print(f"Length of RawInnerRings: {len(df)}")

# Initialize variables to track the number of rows deleted and added
rows_deleted = 0
rows_added = 0

# Lists to keep track of rows to delete and rows to add
indices_to_delete = []
rows_to_add = []

# Collect the indices to delete and the new rows to add
for s, e, path_index in zip(start, end, unique_path_indices):
    print(f"Adding rows from index {s + 1} to {e}")

    # Collect indices to delete
    indices_to_delete.extend(range(s + 1, e + 1))
    
    print("These records will be removed")
    print(f"From record {s + 1} to {e}:")
    print(df.iloc[s + 1:e + 1][['X', 'Y']])
    
    rows_deleted += (e - s)

    print("These records will be added in their place")
    print(f"Path_Index {path_index}:")
    print("  Arc_X values:", paths[path_index]['Arc_X'])
    print("  Arc_Y values:", paths[path_index]['Arc_Y'])

    # Prepare new rows with the correct format
    for arc_x, arc_y in zip(paths[path_index]['Arc_X'], paths[path_index]['Arc_Y']):
        new_row = df.iloc[0].copy()  # Create a new row with the same structure as the original dataframe
        new_row['X'] = arc_x
        new_row['Y'] = arc_y
        rows_to_add.append(new_row)

    rows_added += len(paths[path_index]['Arc_X'])

# Remove the rows to be deleted in reverse order to avoid index shifting
for index in sorted(indices_to_delete, reverse=True):
    df.drop(index, inplace=True)

# Insert new rows in the correct order
for new_row in rows_to_add:
    df = pd.concat([df, pd.DataFrame([new_row])], ignore_index=True)

# Ensure the dataframe is sorted by the index to maintain order
df.sort_index(inplace=True)

# Print the final lengths for debugging
print(f"Final length of new_df: {len(df)}")
print(f"Total rows deleted: {rows_deleted}")
print(f"Total rows added: {rows_added}")

# Write the updated DataFrame to a new sheet in the same Excel file
with pd.ExcelWriter(xlsx_file_path, mode='a', if_sheet_exists='replace') as writer:
    df.to_excel(writer, sheet_name='UpdatedPath', index=False)

print("Updated dataframe has been written to the new sheet 'UpdatedPath'.")
