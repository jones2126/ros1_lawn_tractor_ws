# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/boustrophedon/path_test_plot_multiple_rings.py
import matplotlib.pyplot as plt
import csv

# File paths for the CSV files
file_paths = [
    '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_0.csv',
    '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_1.csv',
    '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_2.csv',
    '/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/Site_01_ring_3.csv'
]

# Colors for each file
colors = ['red', 'green', 'blue', 'yellow']

# Plot each file in a different color
for file_path, color in zip(file_paths, colors):
    x, y = [], []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        for row in reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
    plt.plot(x, y, color=color, label=file_path.split('/')[-1])

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Plots of Rings in Different Colors')
plt.legend()
plt.show()
