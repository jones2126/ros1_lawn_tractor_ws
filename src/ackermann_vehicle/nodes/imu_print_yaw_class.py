#!/usr/bin/env python3

'''
The program subscribes to an IMU (Inertial Measurement Unit) topic and processes the orientation data 
to calculate the current heading and angular velocity around the z-axis. It uses the 
euler_from_quaternion() function from the tf.transformations module to convert the orientation data 
from quaternion to Euler angles, and then computes the change in yaw angle (delta_yaw) and the elapsed 
time between consecutive IMU readings. Based on these values, it calculates the angular velocity around 
the z-axis and updates the current heading angle (self.heading_radians). The program also handles edge cases 
where the yaw angle wraps around from pi to -pi and vice versa, and prints the current heading and angular 
velocity only if the angular velocity is greater than a threshold value (MIN_ANGULAR_VEL_Z). 
The time.monotonic() function is used since it is less subject to changes by the OS.
'''

import rospy
from sensor_msgs.msg import Imu
import math
from tf.transformations import euler_from_quaternion
from time import time
from math import pi
PI = math.pi
MIN_ANGULAR_VEL_Z = 0.005

class IMUSubscriber:
    def __init__(self):
        rospy.init_node('imu_subscriber', anonymous=True)
        rospy.loginfo("imu_subscriber node started...")
        self.heading_radians = -1.488  
        self.prev_yaw = 0.0
        self.prev_time = time.monotonic()

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.spin()

    def imu_callback(self, data):
        orientation = data.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        current_time = time.monotonic()
        elapsed_time = (current_time - self.prev_time)
        delta_yaw = yaw - self.prev_yaw

        if delta_yaw   > PI:
            delta_yaw = delta_yaw  - ( 2 * PI)
        elif delta_yaw  < -PI:
            delta_yaw  = delta_yaw + (2 * PI)
        else:
            delta_yaw = delta_yaw 

        angular_vel_z = delta_yaw / elapsed_time
        self.heading_radians = self.heading_radians + delta_yaw
        if self.heading_radians   > PI:
            self.heading_radians = self.heading_radians  - ( 2 * PI)
        elif self.heading_radians  < -PI:
            self.heading_radians  = self.heading_radians + (2 * PI)
        else:
            self.heading_radians = self.heading_radians  
        '''
        I'm using this as a debug process.  I only print the the data if the IMU is rotating.
        '''                   
        if abs(angular_vel_z) > MIN_ANGULAR_VEL_Z:
            rospy.loginfo("current heading: {:.3f} rad; "\
                            "angular_vel_z: {:.3f} rad/s; "\
                            "delta_yaw: {:.3f} rad"\
                            .format(self.heading_radians, angular_vel_z, delta_yaw))
        self.prev_yaw = yaw
        self.prev_time = current_time

if __name__ == '__main__':
    imu_subscriber = IMUSubscriber()