import pandas as pd
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/dubins/test_reading_spreadsheet.py
ods_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_path_with_intercepts.ods'
df = pd.read_excel(ods_file_path, engine='odf', header=None)  # Reading the .ods file
# Extracting data from columns J and K (which are 9th and 10th columns, as indexing starts from 0)
column_j_data = df.iloc[:, 9]
column_k_data = df.iloc[:, 10]

# print("Column J Data:\n", column_j_data)
# print("Column K Data:\n", column_k_data)
# Keeping only rows where both columns J and K have non-NaN values
df_filtered = df.dropna(subset=[9, 10])
#print(df_filtered[[9, 10]])

# Excluding the first row and converting the DataFrame to a list of tuples
coordinates = [(row[9], row[10]) for index, row in df_filtered.iterrows() if index > 0]

# Print the coordinates list to verify
print(coordinates)
print('length of coordinates:', len(coordinates))