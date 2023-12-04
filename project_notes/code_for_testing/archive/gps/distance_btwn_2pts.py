# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps/distance_btwn_2pts.py
from geopy.distance import geodesic

# Define the latitude and longitude points
# Average Latitude: 40.485288559081575, Average Longitude: -80.33276430378729 @ 1:54 yesterday
# point_1 = (40.485288559081575, -80.33276430378729)

# Average Latitude: 40.48528738008887, Average Longitude: -80.33276336552956 - yesterday's last run
# point_2 = (40.48528738008887, -80.33276336552956)

# # Average Latitude: 40.48528795066455, Average Longitude: -80.33276297432118 @ 1:36
# point_2 = (40.48528795066455, -80.33276297432118)

# origin in locations databse 40.4852469	-80.3327209
point_1 = (40.4852469, -80.3327209)

# Average Latitude: 40.485284877265975, Average Longitude: -80.33262569339004
# File: /home/tractor/bagfiles/Collins_Dr_62_Site_01_run2_2023-11-01-14-51-56.bag Starting at: 1 Until: 95
point_2 = (40.485284877265975, -80.33262569339004)

# Calculate the distance
distance = geodesic(point_1, point_2).meters
print(f"Distance {round(distance, 3)} meters, {round(distance*39.3701, 1)} inches")
