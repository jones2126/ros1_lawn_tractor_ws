# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_calc_angles_for_generator.py
# /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_calc_angles_for_generator.py
import csv
import math
import pandas as pd
print(f"Starting program...")
# Define the function to calculate the ROS angle
def calculate_ROS_angle(x1, y1, x2, y2):
    '''
    Calculates the directional angle with respect to the positive X-axis. This is in line with the ROS REP 103 standard, where an angle 
    of 0 radians corresponds to movement directly along the positive X-axis and approximately 1.57 radians corresponds to movement 
    directly along the positive Y-axis.
    '''    
    angle = math.atan2(y2 - y1, x2 - x1)
    if angle < 0:  # Ensures the angle is between 0 and 2*pi
        angle += 2 * math.pi
    return angle

# coordinates = [
#     (-10.000, -4.000),
#     (-2.860,  -6.678),
#     (-10.470, 15.425),
#     (-9.710,  15.687),
#     (-1.559,  -7.985),
#     (-0.798,  -7.723),
#     (-11.315, 22.819),
#     (-10.554, 23.081),
#     (2.141,   -13.787)
#     ]
    
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



# Initialize an empty list to store (x, y, angle) tuples
data_with_angles = []

# Calculate angle and append data with the angle for all points except the last one
for i in range(len(coordinates) - 1):
    x1, y1 = coordinates[i]
    x2, y2 = coordinates[i + 1]
    angle = calculate_ROS_angle(x1, y1, x2, y2)
    data_with_angles.append((x1, y1, angle))

# For the last point, we'll assume the angle remains the same as the previous one
data_with_angles.append((coordinates[-1][0], coordinates[-1][1], data_with_angles[-1][2] if data_with_angles else 0))

# Write to a CSV file
# csv_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01.csv'
csv_filename = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_path_first_15_stripes.csv'
print(f"Writing .csv file ...")
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    # Write the header
    writer.writerow(['x', 'y', 'angle'])
    # Write the data
    for row in data_with_angles:
        writer.writerow(row)
