#!/usr/bin/env python
'''
script that reads the lat lon data from the bagfile mentioned about 
between a starting and endinging point in seconds, converts those positions 
to x and y coordinates and then plots them using pyplot.

$python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_review_using_rosbag_and_pyplot2.py

'''
import rosbag
import rospy
import geonav_transform.geonav_conversions as gc
import utm

# 435 Pine Valley Dr
# origin_lat = 40.3452899
# origin_lon = -80.1289039

# 62 Collins Dr
origin_lat = 40.48528569166667
origin_lon = -80.33262521333333


# Get UTM zone for the origin
origin_utm_zone = utm.from_latlon(origin_lat, origin_lon)[2]

bag_path = '/home/tractor/bagfiles/2023-11-01-14-51-56.bag'
bag = rosbag.Bag(bag_path)

start_time = 100
end_time = 690

first_timestamp = None
outside_utm_count = 0
total_messages = 0

for topic, msg, t in bag.read_messages(topics=['/fix']):
    timestamp_seconds = t.to_sec()

    if first_timestamp is None:
        first_timestamp = timestamp_seconds

    relative_time = timestamp_seconds - first_timestamp

    if start_time <= relative_time <= end_time:
        lat = msg.latitude
        lon = msg.longitude

        # Get UTM zone for the current point
        current_utm_zone = utm.from_latlon(lat, lon)[2]

        # Check if UTM zones are different
        if current_utm_zone != origin_utm_zone:
            outside_utm_count += 1
            print(f"relative_time: {relative_time} Lat: {lat}, Lon: {lon} - UTM Zone: {current_utm_zone} (Different from origin's UTM Zone: {origin_utm_zone})")
        
        total_messages += 1

bag.close()

print(f"\nTotal messages: {total_messages}")
print(f"Messages outside origin's UTM zone: {outside_utm_count}")

