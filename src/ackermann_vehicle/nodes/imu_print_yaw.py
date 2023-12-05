#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import math
from tf.transformations import euler_from_quaternion
from time import time
rospy.init_node('imu_subscriber', anonymous=True)
rospy.loginfo("imu_subscriber node started...")

prev_yaw = 0.0
prev_time = time()

def imu_callback(data):
    global prev_yaw, prev_time    
    orientation = data.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    current_time = time()
    elapsed_time = (current_time - prev_time)
    delta_yaw = yaw - prev_yaw
    angular_vel_z = delta_yaw / elapsed_time
    if abs(angular_vel_z) > 0.005:
        rospy.loginfo("yaw angle: {:.3f} radians; angular_vel_z: {:.3f} rad/s".format(yaw, angular_vel_z))
    prev_yaw = yaw
    prev_time = current_time       
  

def imu_subscriber():
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    imu_subscriber()
