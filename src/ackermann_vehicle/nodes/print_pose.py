#!/usr/bin/env python3
'''
print_pose.py

This subscribes to odom and prints x_odom, y_odom, yaw, quat_x, quat_y, quat_z, quat_w
"yaw" is calculated using euler_from_quaternion

# ref: https://answers.ros.org/question/10697/how-to-subscribe-to-odom-properly-in-python/
# ref: The Construct https://www.youtube.com/watch?v=mFpH9KK7GvI&feature=youtu.be
# ref: https://answers.ros.org/question/69754/quaternion-transformations-in-python/
'''
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

quat_x = quat_y = quat_z = quat_w = x_pose = y_pose = roll = pitch = yaw = 0.0

def odometryCb(msg):
    global x_pose, y_pose
    # print("in callback")
    # print(msg.pose.pose)
    x_pose = msg.pose.pose.position.x
    y_pose = msg.pose.pose.position.y
    get_rotation(msg)

def get_rotation(msg):
    global roll, pitch, yaw, quat_x, quat_y, quat_z, quat_w
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)    
    quat_x = msg.pose.pose.orientation.x
    quat_y = msg.pose.pose.orientation.y
    quat_z = msg.pose.pose.orientation.z
    quat_w = msg.pose.pose.orientation.w

# main function
rospy.init_node('check_odometry')
sub = rospy.Subscriber ('/odom', Odometry, odometryCb)
r = rospy.Rate(1)
while not rospy.is_shutdown():
    print(x_pose, y_pose, yaw, quat_x, quat_y, quat_z, quat_w)
    r.sleep()