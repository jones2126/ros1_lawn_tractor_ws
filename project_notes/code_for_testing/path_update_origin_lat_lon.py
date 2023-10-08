#!/usr/bin/env python

# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_update_origin_lat_lon.py

import rospy
from sensor_msgs.msg import NavSatFix
import time
import math

# Initialize node
rospy.init_node('average_position', anonymous=True)

# Set reference latitude and longitude
ref_lat = 40.34530756451623
ref_lon = -80.12885480045905

# Set the origin latitude and longitude
origin_lat = 40.34534080
origin_lon = -80.12894600

# Initialize variables
lat_sum = 0
lon_sum = 0
count = 0
start_time = time.time()

def callback(data):
    global lat_sum, lon_sum, count, start_time
    
    # Check if the fix status is 2
    if data.status.status == 2:
        lat_sum += data.latitude
        lon_sum += data.longitude
        count += 1
    
    # Check if 30 seconds have passed
    if time.time() - start_time >= 30:
        # Unsubscribe to prevent further callbacks
        sub.unregister()
        
        # Calculate the average latitude and longitude
        avg_lat = lat_sum / count if count else 0
        avg_lon = lon_sum / count if count else 0
        
        # Calculate the delta from the reference point
        delta_lat = avg_lat - ref_lat
        delta_lon = avg_lon - ref_lon
        
        # Apply the delta to the origin
        updated_origin_lat = origin_lat + delta_lat
        updated_origin_lon = origin_lon + delta_lon
        
        # Print the updated origin latitude and longitude
        rospy.loginfo("rosparam set GPS_origin_lat %s", updated_origin_lat)
        rospy.loginfo("rosparam set GPS_origin_lon %s", updated_origin_lon)

  #614  rosparam set GPS_origin_lat 40.345298
  #613  rosparam set GPS_origin_lon -80.128946
  # data: [7.7163467918289825, -3.5679386565461755,
  # data: [7.769423488527536, -3.595980806276202,
  # data: [7.778599630459212, -3.611017727293074
# data: [7.778599630459212, -3.611017727293074


        # Print ending message
        rospy.loginfo("Averaging and updating completed. Shutting down the node.")
        rospy.signal_shutdown("Task completed.")

# Subscribe to the /fix topic
sub = rospy.Subscriber('/fix', NavSatFix, callback)

# Print starting message
rospy.loginfo("Starting to average latitude and longitude for 30 seconds from /fix topic.")


# Keep the node running
rospy.spin()
