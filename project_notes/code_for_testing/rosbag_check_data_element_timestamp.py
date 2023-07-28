#!/usr/bin/env python3
'''
Reads a bag file, and for each message on the /my_array_topic topic, checks if the 13th element of the 
array (index 12 in Python's 0-based indexing) is 0, and if so, prints out the Unix timestamp 
of the message:

to run 
$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/rosbag_check_data_element_timestamp.py
'''

import rosbag
from datetime import datetime

bag = rosbag.Bag('/home/tractor/bagfiles/2023-07-26-10-14-17.bag')

for topic, msg, t in bag.read_messages(topics=['/my_array_topic']):
    # Check if the 13th element of the array is 0
    if msg.data[12] == 0:
        # Print the Unix timestamp
        unix_timestamp = t.to_sec()
        print("Unix timestamp: ", unix_timestamp)

bag.close()