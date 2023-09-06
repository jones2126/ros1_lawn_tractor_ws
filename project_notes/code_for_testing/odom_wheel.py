#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

WHEELBASE = 1.0  # Set the wheelbase of your robot

class AckermannOdometry:
    def __init__(self):
        rospy.init_node('ackermann_odometry')
        
        self.left_meters_travelled = 0
        self.right_meters_travelled = 0
        self.yaw = 182  # Initial orientation in degrees
        self.x = 0
        self.y = 0

        self.position_covariance = [1e-3, 0, 0,
                                    0, 1e-3, 0,
                                    0, 0, 1e-6]  # Adjust these values according to the actual uncertainties

        self.orientation_covariance = [1e-6, 0, 0,
                                       0, 1e-6, 0,
                                       0, 0, 1e-3]  # Adjust these values according to the actual uncertainties

        self.imu_orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # Initialize with zeros
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        rospy.Subscriber('/left_meters_travelled', Float64, self.left_callback)
        rospy.Subscriber('/right_meters_travelled', Float64, self.right_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.update_odometry()

    def build_covariance_matrix(self, pos_cov, ori_cov):
        covariance = [0] * 36
        for i in range(3):
            for j in range(3):
                covariance[i * 6 + j] = pos_cov[i * 3 + j]  # Position covariance (top-left 3x3 block)
                covariance[(i + 3) * 6 + (j + 3)] = ori_cov[i * 3 + j]  # Orientation covariance (bottom-right 3x3 block)
        return covariance


    def left_callback(self, msg):
        self.left_meters_travelled = msg.data

    def right_callback(self, msg):
        self.right_meters_travelled = msg.data

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw_rad = euler_from_quaternion(orientation_list)
        self.yaw = math.degrees(yaw_rad)
        
        # Store the orientation covariance
        self.imu_orientation_covariance = msg.orientation_covariance

    def update_odometry(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            delta_meters = (self.right_meters_travelled + self.left_meters_travelled) / 2.0
            delta_yaw_rad = math.radians(self.yaw)

            self.x += delta_meters * math.cos(delta_yaw_rad)
            self.y += delta_meters * math.sin(delta_yaw_rad)

            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'odom_wheel'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            quaternion = quaternion_from_euler(0, 0, delta_yaw_rad)
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]

            odom.pose.covariance = self.build_covariance_matrix(self.position_covariance, self.imu_orientation_covariance)

            self.odom_pub.publish(odom)
            rate.sleep()

if __name__ == '__main__':
    try:
        AckermannOdometry()
    except rospy.ROSInterruptException:
        pass
