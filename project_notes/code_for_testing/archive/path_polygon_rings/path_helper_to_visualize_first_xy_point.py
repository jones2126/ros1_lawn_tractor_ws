# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/path_helper_to_visualize_first_xy_point.py

# import matplotlib.pyplot as plt
# import pandas as pd

# csv_input_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_04.csv'
# # Load your CSV file
# data = pd.read_csv(csv_input_file_path)  # Replace with the path to your CSV file

# # Plotting the points
# plt.figure(figsize=(10, 6))
# plt.scatter(data['X'], data['Y'], color='blue')

# # Annotating each point with its index
# for i, point in data.iterrows():
#     plt.text(point['X'], point['Y'], str(i), fontsize=8, ha='right')

# # Setting the plot title and labels
# plt.title('Scatter Plot of Points')
# plt.xlabel('X Coordinate')
# plt.ylabel('Y Coordinate')

# plt.axis('equal')
# plt.show()


import matplotlib.pyplot as plt
import pandas as pd

#csv_input_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_04.csv'
csv_input_file_path = '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_04_ring_3.csv'

# Load your CSV file
data = pd.read_csv(csv_input_file_path)  # Replace with the path to your CSV file

# Define the step variable
step = 2  # Change this value to 3 or any other integer to plot every 'n-th' point

# Plotting every 'n-th' point where 'n' is the step value
plt.figure(figsize=(10, 6))
#plt.scatter(data['lng'][::step], data['lat'][::step], color='blue')  # Use 'lng' and 'lat' columns with the step variable
plt.scatter(data['X'][::step], data['Y'][::step], color='blue')  # Use 'lng' and 'lat' columns with the step variable

# Annotating every 'n-th' point with its index
for i, point in data[::step].iterrows():  # Use the step variable in the loop as well
    #plt.text(point['lng'], point['lat'], str(i), fontsize=6, ha='right')
    plt.text(point['X'], point['Y'], str(i), fontsize=6, ha='right')

# Setting the plot title and labels
plt.title('Scatter Plot of Every nth Point')
plt.xlabel('Longitude')
plt.ylabel('Latitude')

plt.axis('equal')
plt.show()

