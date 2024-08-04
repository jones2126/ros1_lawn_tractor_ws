#!/usr/bin/env python3
# python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/utilities/calc_size_of_myArrayTopic.py ~/bagfiles/archive/2024-07-02-13-14-15.bag
# ~ros1_lawn_tractor_ws/project_notes/code_for_testing/utilities

import rosbag
import sys
from std_msgs.msg import Float64MultiArray
from collections import deque
import struct

def calculate_message_size(data):
    # Calculate size based on the actual float64 values
    return len(data) * 8  # 8 bytes per float64

def analyze_bag(bag_file):
    total_size = 0
    message_count = 0
    first_five = []
    last_five = deque(maxlen=5)

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/my_array_topic']):
            # Calculate the size of the message
            message_size = calculate_message_size(msg.data)
            total_size += message_size
            message_count += 1

            # Store first five messages
            if len(first_five) < 5:
                first_five.append((message_count, list(msg.data), message_size))

            # Store last five messages
            last_five.append((message_count, list(msg.data), message_size))

    if message_count > 0:
        average_size = total_size / message_count
        print(f"Total messages: {message_count}")
        print(f"Average message size: {average_size:.2f} bytes")
        
        print("\nFirst five records:")
        for count, data, size in first_five:
            print(f"Message {count}: {data}")
            print(f"Size: {size} bytes")
        
        print("\nLast five records:")
        for count, data, size in last_five:
            print(f"Message {count}: {data}")
            print(f"Size: {size} bytes")
    else:
        print("No messages found in the topic /my_array_topic")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script_name.py path_to_your_bag_file.bag")
        sys.exit(1)

    bag_file = sys.argv[1]
    analyze_bag(bag_file)