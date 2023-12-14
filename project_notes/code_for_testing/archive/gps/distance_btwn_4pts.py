# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps/distance_btwn_4pts.py

import matplotlib.pyplot as plt
import geopy.distance

# Define the coordinates
coords = [
    (40.48522252, -80.332622595),              # 15:34
    (40.48522116666667, -80.33264353666667),   # 15:44
    (40.485219586666666, -80.33264322166667),  # 15:57
    (40.485219611666665, -80.33264243666666),  # 16:07
    (40.485284877266, -80.3326256933901)       # presumed correct
]

# Plotting the points
plt.figure(figsize=(10, 8))
for i, coord in enumerate(coords):
    plt.plot(coord[1], coord[0], 'o', label=f'Point {i+1}')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Plot of the Given Coordinates')
plt.legend()
plt.grid(True)

# Calculating distances in inches
distances_in_inches = []
for i in range(len(coords)-1):
    distance_km = geopy.distance.distance(coords[i], coords[i+1]).km
    distances_in_inches.append(distance_km * 39370.079)  # 1 km = 39370.079 inches

for i, dist in enumerate(distances_in_inches, 1):
    print(f"Distance between point {i} and point {i+1}: {dist} inches")

plt.axis('equal')  # Ensure equal aspect ratio
plt.show(), distances_in_inches


