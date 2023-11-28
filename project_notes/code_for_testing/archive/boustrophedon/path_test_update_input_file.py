# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_update_input_file.py
import pandas as pd

import os
script_name = os.path.basename(__file__)
print(f"running script: {script_name}")

column_indexes = [11, 12, 13]  # Columns for 'start_x', 'start_y', 'angle'; Remember Col A is zero, B is 1.
starting_row_of_data = 3
rows_of_data = 101

# File paths
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophedon_w_formulas_v1.ods'
txt_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_boustrophen_generator_input.txt'

# Read the spreadsheet file using column indexes
df = pd.read_excel(ods_file_path, engine='odf', sheet_name='Site_01_path_with_intercepts', usecols=column_indexes, skiprows=(starting_row_of_data-1), nrows=rows_of_data)

# Write to the txt_file_path file
with open(txt_file_path, 'w') as file:
    for index, row in df.iterrows():
        file.write(f"{row.iloc[0]} {row.iloc[1]} {row.iloc[2]}\n")

print("Data has been successfully copied to the .txt file.")
print("Next run the generator program.")
