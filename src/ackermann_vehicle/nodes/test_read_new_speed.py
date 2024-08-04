#!/usr/bin/env python3
# test_read_new_speed.py
# rosrun rosserial_python serial_node.py _port:=/dev/odom_right _baud:=115200
# /dev/odom_left 
# $ python3 /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/test_read_new_speed.py

import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):
    # Extract the speed data (assuming it's the fourth element in the array)
    totalRotations = round(data.data[1], 3)
    delta = round(data.data[2], 1)
    speed = round(data.data[3], 3)
    rospy.loginfo(f"Total Rotations: {totalRotations:.3f}, Delta: {delta:.1f}, Speed: {speed:.3f}")




def listener():
    # Initialize the ROS node
    rospy.init_node('wheel_data_listener', anonymous=True)

    # Create a subscriber to the 'wheel_data_left' topic
    rospy.Subscriber('wheel_data_left', Float32MultiArray, callback)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    listener()
