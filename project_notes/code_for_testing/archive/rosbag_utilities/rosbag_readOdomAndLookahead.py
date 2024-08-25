#!/usr/bin/env python3
# python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/rosbag_readOdomAndLookahead.py
#!/usr/bin/env python
'''
I had a plotjuggler screen shot based on a rosbag captured driving my outdoor robot. There are extreme 
odom positions that are illogical (e.g. x is < -50 or > 20). Can you provide a script that 
reads /home/tractor/bagfiles/2024-08-09-11-41-55.bag and looks for extreme values in the topics look ahead 
and odom pose? The script should also print the time delta from the start of the rosbag which is how plotjuggler 
displays time.
'''

import rosbag
import rospy

# Path to the ROS bag file
bag_path = '/home/tractor/bagfiles/2024-08-09-11-41-55.bag'
bag = rosbag.Bag(bag_path)

# Thresholds for extreme values
x_min_threshold = -50
x_max_threshold = 20

# Containers to store time deltas and extreme values
extreme_values_odom = []
extreme_values_lookahead = []

# Time range for reading messages (if needed, you can set specific times)
start_time = 0
end_time = float('inf')

first_timestamp = None

for topic, msg, t in bag.read_messages(topics=['/odom', '/lookahead_data']):
    timestamp_seconds = t.to_sec()

    if first_timestamp is None:
        first_timestamp = timestamp_seconds

    relative_time = timestamp_seconds - first_timestamp

    if start_time <= relative_time <= end_time:
        if topic == '/odom':
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            
            if x < x_min_threshold or x > x_max_threshold:
                extreme_values_odom.append((relative_time, x, y))
        
        if topic == '/lookahead_data':
            x_lookahead = msg.data[0]
            y_lookahead = msg.data[1]
            
            if x_lookahead < x_min_threshold or x_lookahead > x_max_threshold:
                extreme_values_lookahead.append((relative_time, x_lookahead, y_lookahead))

bag.close()

# Print results
print("Extreme Odom Values (time delta, x, y):")
for time_delta, x, y in extreme_values_odom:
    print(f"Time delta: {time_delta:.3f} s, X: {x:.3f}, Y: {y:.3f}")

print("\nExtreme Lookahead Values (time delta, x, y):")
for time_delta, x_lookahead, y_lookahead in extreme_values_lookahead:
    print(f"Time delta: {time_delta:.3f} s, X: {x_lookahead:.3f}, Y: {y_lookahead:.3f}")
