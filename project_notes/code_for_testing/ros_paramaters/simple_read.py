#!/usr/bin/env python3

import rospy
from rospy.exceptions import ROSException

# Initialize the ROS node
rospy.init_node('speed_params_reader')

# Wait for the /speed_params parameter to be available
while not rospy.is_shutdown():
    try:
        speed_params = rospy.get_param('/speed_params')
        break
    except ROSException:
        rospy.loginfo("Waiting for /speed_params parameter...")
        rospy.sleep(1)  # Wait for 1 second before trying again

# Print the parameter values
print("Retrieved speed_params: ", speed_params)