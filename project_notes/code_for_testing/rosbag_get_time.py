#!/usr/bin/env python3
'''
Used to get the time stamp of the first message so I can look at print messages and be able to compare the times
to times referenced in plotjuggler

to run $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/rosbag_get_time.py
'''

import rosbag
from datetime import datetime

bag = rosbag.Bag('/home/tractor/bagfiles/2023-07-26-10-14-17.bag')

for topic, msg, t in bag.read_messages():
    # Convert the ROS timestamp to a datetime object
    timestamp = datetime.fromtimestamp(t.to_sec())
    print("First message time: ", timestamp)

    # Print the Unix timestamp
    unix_timestamp = t.to_sec()
    print("First message Unix timestamp: ", unix_timestamp)

    break  # We only want the first message, so we break the loop

bag.close()