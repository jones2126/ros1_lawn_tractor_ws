#!/usr/bin/env python3

# $ python3 /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/path_update_origin_lat_lon.py

import subprocess
import rospy
from sensor_msgs.msg import NavSatFix
import time
import math

# Initialize node
rospy.init_node('average_position', anonymous=True)


# Set reference latitude and longitude
ref_lat = 40.48524688166667  #62 Collins Dr
ref_lon = -80.332720941667

# Set the origin latitude and longitude
origin_lat = 40.48524688166667  #62 Collins Dr
origin_lon = -80.332720941667

# Initialize variables
lat_sum = 0
lon_sum = 0
count = 0
start_time = None

def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of Earth in kilometers
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat/2) * math.sin(dlat/2) +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon/2) * math.sin(dlon/2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c
    return distance

def callback(data):
    global lat_sum, lon_sum, count, start_time

    if start_time is None and data.status.status == 2:
        start_time = time.time()
        rospy.loginfo("Started averaging lat/lon at %s", start_time)
    
    # Check if the fix status is 2
    if data.status.status == 2:
        lat_sum += data.latitude
        lon_sum += data.longitude
        count += 1
    
    # Check if 30 seconds have passed
    if start_time is not None and time.time() - start_time >= 30:
        # Unsubscribe to prevent further callbacks
        sub.unregister()
        
        # Calculate the average latitude and longitude
        avg_lat = lat_sum / count if count else 0
        avg_lon = lon_sum / count if count else 0
        print('writing average_position')
        with open('average_position_lat_and_lon.txt', 'w') as file:
            file.write(f"avg_lat: {avg_lat}\navg_lon: {avg_lon}\n")    

        rospy.loginfo("avg_lat %s", avg_lat)
        rospy.loginfo("avg_lon %s", avg_lon)

        # Calculate distance in kilometers
        distance_km = haversine(avg_lat, avg_lon, origin_lat, origin_lon)
        distance_feet = distance_km * 3280.84

        print(f"Distance between current and ref location (ft): {distance_feet:.2f} feet")                 
        
        # Calculate the delta from the reference point
        delta_lat = avg_lat - ref_lat
        delta_lon = avg_lon - ref_lon
        
        # Apply the delta to the origin
        updated_origin_lat = origin_lat + delta_lat
        updated_origin_lon = origin_lon + delta_lon
        
        # Print the updated origin latitude and longitude

        rospy.loginfo("origin_lat %s", origin_lat)
        rospy.loginfo("origin_lon %s", origin_lon)

        rospy.loginfo("delta_lat %s", delta_lat)
        rospy.loginfo("delta_lon %s", delta_lon)        

        rospy.loginfo("updated origin_lat %s", updated_origin_lat)
        rospy.loginfo("updated origin_lon %s", updated_origin_lon)

        # Execute the commands
       # subprocess.run(["rosparam", "set", "GPS_origin_lat", str(updated_origin_lat)], check=True)
       # subprocess.run(["rosparam", "set", "GPS_origin_lon", str(updated_origin_lon)], check=True)        

        # Print ending message
        rospy.loginfo("Averaging and updating completed. Shutting down path_update_origin_lat_lon.py")
        rospy.signal_shutdown("Task completed.")

# Subscribe to the /fix topic
sub = rospy.Subscriber('/fix', NavSatFix, callback)

# Print starting message
rospy.loginfo("Starting to average latitude and longitude for 30 seconds from /fix topic.")


# Keep the node running
rospy.spin()
