#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi
import time

PI = pi

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.left_distance_prev = 0.0
        self.right_distance_prev = 0.0
        self.left_delta = 0.0
        self.right_delta = 0.0
        self.x = 0.0
        self.y = 0.0
        self.heading_radians_imu = -1.509
        self.heading_radians_wheels = self.heading_radians_imu
        self.prev_yaw = 0.0
        self.angular_vel_z_imu = 0.0
        self.angular_vel_z_wheel = 0.0
        self.imu_calls = 0
        self.imu_skips = 0
        self.quat = quaternion_from_euler(0.0, 0.0, self.heading_radians_imu)
        self.wheelbase = 1.27
        self.last_time = rospy.Time.now()
        self.prev_time_imu = rospy.Time.now()
        self.last_print_time = rospy.Time.now()

        rospy.Subscriber('/left_meters_travelled_msg', Float32, self.left_distance_cb)
        rospy.Subscriber('/right_meters_travelled_msg', Float32, self.right_distance_cb)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.radian1_pub = rospy.Publisher('radian1', Float64, queue_size=10)
        self.radian2_pub = rospy.Publisher('radian2', Float64, queue_size=10)        
        
    def left_distance_cb(self, msg):
        self.left_distance = msg.data
                      
    def right_distance_cb(self, msg):
        self.right_distance = msg.data

    def imu_callback(self, msg):
        self.imu_calls = self.imu_calls + 1
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        current_time_imu = rospy.Time.now()
        delta_time_imu_raw = current_time_imu - self.prev_time_imu
        delta_time_imu = delta_time_imu_raw.to_sec()
    
        if delta_time_imu_raw.to_nsec() < 1:  # check if the time difference is less than 1 nanosecond
            #rospy.loginfo("Ignoring callback due to very small time difference")
            self.imu_skips = self.imu_skips + 1
            return

        delta_yaw = yaw - self.prev_yaw

        if delta_yaw   > PI:
            delta_yaw = delta_yaw  - ( 2 * PI)
        elif delta_yaw  < -PI:
            delta_yaw  = delta_yaw + (2 * PI)
        else:
            delta_yaw = delta_yaw 
        self.heading_radians_imu = self.heading_radians_imu + delta_yaw

        # check if zero otherwise calculate angular z            
        if delta_time_imu == 0:
            self.angular_vel_z_imu = 0
            rospy.loginfo("delta_time_imu = 0")
            rospy.loginfo("current_time_imu: %s, prev_time_imu: %s, delta_time_imu: %s", \
                          current_time_imu, self.prev_time_imu, delta_time_imu)            
        else:
            self.angular_vel_z_imu = delta_yaw / delta_time_imu

        if self.heading_radians_imu   > PI:
            self.heading_radians_imu = self.heading_radians_imu  - ( 2 * PI)
        elif self.heading_radians_imu  < -PI:
            self.heading_radians_imu  = self.heading_radians_imu + (2 * PI)
        else:
            self.heading_radians_imu = self.heading_radians_imu 
        self.quat = quaternion_from_euler(0.0, 0.0, self.heading_radians_imu)
        self.prev_yaw = yaw
        self.prev_time_imu = current_time_imu            
                      
    def publish_odom(self):
        current_time_odom = rospy.Time.now()
        delta_time_odom = (current_time_odom - self.last_time).to_sec()

        # Calculate distance travelled by each wheel - there will be a rollover event that is not programmed
        self.left_delta = self.left_distance - self.left_distance_prev
        self.right_delta = self.right_distance - self.right_distance_prev

        # Calculate the distance travelled and speed by the robot using the average distance of the two wheels
        distance = (self.left_delta + self.right_delta) / 2
        if delta_time_odom == 0:
            linear_velocity = 0
            rospy.loginfo("delta_time_odom = 0")
        else:
            linear_velocity = distance / delta_time_odom        

        # Calculate the change in orientation (theta) of the robot to be used in the distance calculation
        delta_theta = (self.right_delta - self.left_delta) / self.wheelbase
        self.heading_radians_wheels += delta_theta

        if self.heading_radians_wheels   > PI:
            self.heading_radians_wheels = self.heading_radians_wheels  - ( 2 * PI)
        elif self.heading_radians_wheels  < -PI:
            self.heading_radians_wheels  = self.heading_radians_wheels + (2 * PI)
        else:
            self.heading_radians_wheels = self.heading_radians_wheels 

        # Calculate the change in x and y position of the robot
        #delta_x = distance * cos(self.heading_radians_wheels)
        #delta_y = distance * sin(self.heading_radians_wheels)
        delta_x = distance * cos(self.heading_radians_imu)
        delta_y = distance * sin(self.heading_radians_imu)               
        self.x += delta_x
        self.y += delta_y

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time_odom
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = self.quat[0]  # currently defined by IMU, not GPS or wheels
        odom_msg.pose.pose.orientation.y = self.quat[1]
        odom_msg.pose.pose.orientation.z = self.quat[2]
        odom_msg.pose.pose.orientation.w = self.quat[3]
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel_z_imu

        self.odom_pub.publish(odom_msg)

        self.radian1_pub.publish(self.heading_radians_imu)
        self.radian2_pub.publish(self.heading_radians_wheels)        

        # Broadcast transform between odom and base_footprint frames
        br = TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = current_time_odom
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.quat[0]  # currently defined by IMU, not GPS or wheels
        t.transform.rotation.y = self.quat[1]
        t.transform.rotation.z = self.quat[2]
        t.transform.rotation.w = self.quat[3]

        br.sendTransform(t)

        self.left_distance_prev = self.left_distance
        self.right_distance_prev = self.right_distance
        self.last_time = current_time_odom

    def print_info(self, event=None):
        rospy.loginfo("dist trvld left: {:.1f} ; right: {:.1f} ; delta left: {:.1f} ; right: {:.1f} ; "\
                      "heading: {:.2f} ; angular_vel_z_imu: {:.3f}"\
                      .format(self.left_distance, self.right_distance, self.left_delta, self.right_delta, \
                              self.heading_radians_imu, self.angular_vel_z_imu))
        rospy.loginfo("imu calls: {:d} ; imu skips: {:d}".format(self.imu_calls, self.imu_skips))
        self.imu_calls = 0
        self.imu_skips = 0      

if __name__ == '__main__':
    odom_pub = OdomPublisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        current_time_main = rospy.Time.now()
        time_elapsed_main = (current_time_main - odom_pub.last_print_time).to_sec()
        odom_pub.publish_odom()
        if time_elapsed_main >= 2.0:  # call print_info() every 2 seconds            
            odom_pub.print_info()
            odom_pub.last_print_time = rospy.Time.now()
        rate.sleep()