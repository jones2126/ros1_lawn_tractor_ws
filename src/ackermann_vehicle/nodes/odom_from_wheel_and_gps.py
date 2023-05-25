#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi
import time

# Import geonav tranformation module
import sys
sys.path.append('/home/tractor/catkin_ws/src/geonav_transform/src/')
import geonav_transform.geonav_conversions as gc
from imp import reload
reload(gc)

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
        self.left_speed = 0
        self.right_speed = 0
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
        #self.GPS_origin_lat = 40.345245345   # should represent garage at 435 Pine Valley Dr
        #self.GPS_origin_lon = -80.128990477
        self.GPS_origin_lat = 40.345317290728474
        self.GPS_origin_lon = -80.12893737841993

        self.x = 0.0
        self.y = 0.0        
        self.x_gps = 0.0
        self.y_gps = 0.0
        self.x_wheel = 0.0
        self.y_wheel = 0.0
        self.RTK_fix = False
        self.non_RTK_fix = 0   

        rospy.Subscriber('/left_meters_travelled_msg', Float32, self.left_distance_cb)
        rospy.Subscriber('/right_meters_travelled_msg', Float32, self.right_distance_cb)

        rospy.Subscriber('/left_speed', Float32, self.left_speed_cb)
        rospy.Subscriber('/right_speed', Float32, self.right_speed_cb)

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.hdg_from_imu_pub = rospy.Publisher('hdg_from_imu', Float64, queue_size=1)
        self.hdg_from_wheels_pub = rospy.Publisher('hdg_from_wheels', Float64, queue_size=1)        
        
    def left_distance_cb(self, msg):
        self.left_distance = msg.data
                      
    def right_distance_cb(self, msg):
        self.right_distance = msg.data

    def left_speed_cb(self, msg):
        self.left_speed = msg.data

    def right_speed_cb(self, msg):
        self.right_speed = msg.data               

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
            self.imu_skips = self.imu_skips + 1  # keep track to monitor how often this is happening
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
            # I was having some issues with self.angular_vel_z_imu being way out of bounds
            # this is a stop gap measure until I can figure out why
            if self.angular_vel_z_imu > PI:
                self.angular_vel_z_imu = PI
            elif self.angular_vel_z_imu < -PI:
                self.angular_vel_z_imu = -PI

        if self.heading_radians_imu   > PI:
            self.heading_radians_imu = self.heading_radians_imu  - ( 2 * PI)
        elif self.heading_radians_imu  < -PI:
            self.heading_radians_imu  = self.heading_radians_imu + (2 * PI)
        else:
            self.heading_radians_imu = self.heading_radians_imu 
        self.quat = quaternion_from_euler(0.0, 0.0, self.heading_radians_imu)
        self.prev_yaw = yaw
        self.prev_time_imu = current_time_imu

    def gps_callback(self, data):
        # Check to see if we are in GPS FIX mode
        if data.status.status != 2:
            self.non_RTK_fix = self.non_RTK_fix  + 1
            #rospy.loginfo("Bad GPS status - data.status.status: %s", data.status.status)   # for debugging
            self.RTK_fix = False    
        if data.status.status == 2:
            #Convert from lat/lon to x/y
            self.x_gps, self.y_gps = gc.ll2xy(data.latitude, data.longitude, self.GPS_origin_lat, self.GPS_origin_lon)
            self.RTK_fix = True
            self.non_RTK_fix = 0
                      
    def publish_odom(self):
        current_time_odom = rospy.Time.now()
        delta_time_odom = (current_time_odom - self.last_time).to_sec()

        # Calculate distance travelled by each wheel - there will be a rollover event that is not programmed yet
        self.left_delta = self.left_distance - self.left_distance_prev
        self.right_delta = self.right_distance - self.right_distance_prev

        # Calculate the distance travelled and speed by the robot using the average distance of the two wheels
        distance = (self.left_delta + self.right_delta) / 2
        linear_velocity_from_wheels = (self.left_speed + self.right_speed) / 2
        '''
        if delta_time_odom == 0: # I was having an issue with this, but added another "nanosecond" check above
            linear_velocity = 0
            rospy.loginfo("delta_time_odom = 0")
        else:
            linear_velocity = distance / delta_time_odom
        '''     

        # Calculate the change in orientation (theta) of the robot to be used in the distance calculation
        delta_theta = (self.right_delta - self.left_delta) / self.wheelbase
        self.heading_radians_wheels += delta_theta

        if self.heading_radians_wheels   > PI:
            self.heading_radians_wheels = self.heading_radians_wheels  - ( 2 * PI)
        elif self.heading_radians_wheels  < -PI:
            self.heading_radians_wheels  = self.heading_radians_wheels + (2 * PI)
        else:
            self.heading_radians_wheels = self.heading_radians_wheels 

        # Calculate the change in x and y position of the robot using wheel data
        delta_x = distance * cos(self.heading_radians_imu)
        delta_y = distance * sin(self.heading_radians_imu)               
        self.x_wheel += delta_x
        self.y_wheel += delta_y

        if self.RTK_fix == True:
            self.x = self.x_gps
            self.y = self.y_gps
            self.x_wheel = self.x_gps
            self.y_wheel = self.y_gps
        else:
            self.x = self.x_wheel
            self.y = self.y_wheel
        # Create and publish Odometry message
        # to publish this I need x,y, quat, linear x and angular z
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
        odom_msg.twist.twist.linear.x = linear_velocity_from_wheels
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel_z_imu

        self.odom_pub.publish(odom_msg)

        self.hdg_from_imu_pub.publish(self.heading_radians_imu)
        self.hdg_from_wheels_pub.publish(self.heading_radians_wheels)        

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
        rospy.loginfo("imu calls: {:d} ; imu skips: {:d} ; non_RTK count: {:d} "\
                      .format(self.imu_calls, self.imu_skips, self.non_RTK_fix))
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