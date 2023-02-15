#!/usr/bin/env python
# my_quaternion.py
# reference: The Construct https://www.youtube.com/watch?v=mFpH9KK7GvI&feature=youtu.be
# reference: https://answers.ros.org/question/69754/quaternion-transformations-in-python/

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
rospy.loginfo("start my_quaternion.py")
print("hello world")

roll = pitch = yaw = 0.0

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print(yaw)

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    #quat = quaternion_from_euler (roll, pitch,yaw)
    print(yaw)
    r.sleep()