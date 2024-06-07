#!/usr/bin/env python

'''
Script to build a path of rings and adjust the path to go around the obstacle.

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/build_concentric_polygon_path.py
'''
print("Running build_concentric_polygon_path.py")

import pandas as pd
import matplotlib.pyplot as plt

def process_arc_path(file_path):
    df = pd.read_excel(file_path, sheet_name='ArcPath') # read the points around the obstacle

    # Extract the unique values in the 'Path_Index' column
    unique_path_indices = df['Path_Index'].unique().tolist()

    unique_path_indices.reverse() # Reverse the order of the list because I want to rebuild the list from the bottom up

    # Store obstacle avoidance path, (i.e. Arc_X and Arc_Y values) in lists for each Path_Index
    paths = {}
    for path_index in unique_path_indices:
        path_data = df[df['Path_Index'] == path_index]
        arc_x_values = path_data['Arc_X'].tolist()
        arc_y_values = path_data['Arc_Y'].tolist()
        paths[path_index] = {'Arc_X': arc_x_values, 'Arc_Y': arc_y_values}

    return unique_path_indices, paths

def process_intersection_points(file_path):
    df = pd.read_excel(file_path, sheet_name='IntersectionPoints') # Read the points of intersection sheet   
    row_index_values = df['Row_Index'].tolist()   # Get the values 'Row_Index' values which are the start and end intersect points
    start = row_index_values[::2]  # Load the starting intersection points 
    end = row_index_values[1::2]   # Load the ending intersection points 

    # Reverse the lists because I need to use these references to inject the obstacle avoidance path and 
    # I need to do that from the bottom of the list and work up so the references remain valid.
    start.reverse()
    end.reverse()

    return start, end

def plot_coordinates(file_path):
    df = pd.read_excel(file_path, sheet_name='UpdatedPath')
    
    unique_path_indices = df['Path_Index'].unique()
    plt.figure(figsize=(10, 6))
    
    for path_index in unique_path_indices:
        path_data = df[df['Path_Index'] == path_index]
        plt.plot(path_data['X'].values, path_data['Y'].values, label=f'Path_Index {path_index}')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Plot of X and Y Coordinates with Different Colors for Each Path_Index')
    plt.legend()
    plt.show()

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

# Print the 'X' and 'Y' values for the start and end positions and the Arc_X and Arc_Y values for each unique_path_index
for s, e, path_index in zip(start, end, unique_path_indices):
    print("These records will be removed")
    print(f"From record {s + 1} to {e}:")
    print(df.iloc[s + 1:e + 1][['X', 'Y']])
    
    # Drop records that represent inside the obstacle
    indices_to_delete = df.iloc[s + 1:e + 1].index
    for index in sorted(indices_to_delete, reverse=True):
        df.drop(index, inplace=True)
        
    print("These records will be added in their place")
    
    # Prepare new rows with the correct format
    rows_to_add = []
    for arc_x, arc_y in zip(paths[path_index]['Arc_X'], paths[path_index]['Arc_Y']):
        new_row = df.iloc[0].copy()
        new_row['Path_Index'] = path_index 
        new_row['X'] = arc_x
        new_row['Y'] = arc_y
        rows_to_add.append(new_row)
    
    rows_to_add_df = pd.DataFrame(rows_to_add) # Convert rows_to_add so they can be added to the dataFrame
    print(rows_to_add_df)    
    
    # Separate the dataframe into two pieces so the obstacle avoidance waypoints can be inserted in between them
    df_part1 = df.iloc[:s+1]
    df_part2 = df.iloc[s+1:]
    
    df_part2.index = df_part2.index + len(rows_to_add_df)  # Adjust the index for df_part2 to ensure proper alignment without overwriting
    df = pd.concat([df_part1, rows_to_add_df, df_part2])   # Concatenate the parts
    df = df.reset_index(drop=True)   # Reset the index of the DataFrame to maintain a continuous index

# Write the updated DataFrame to a new sheet in the same Excel file
with pd.ExcelWriter(xlsx_file_path, mode='a', if_sheet_exists='replace') as writer:
    df.to_excel(writer, sheet_name='UpdatedPath', index=False)

print("Updated dataframe has been written to the new sheet 'UpdatedPath'.")

plot_coordinates(xlsx_file_path)