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
import math

from std_msgs.msg import Float64MultiArray

# Import geonav tranformation module
# import sys
# sys.path.append('/home/tractor/catkin_ws/src/geonav_transform/src/')
import geonav_transform.geonav_conversions as gc
# from imp import reload
# reload(gc)

PI = pi

class OdomPublisher:

    def calibrate_lat_lon_origin(self, event):
        GPS_origin_lat = rospy.get_param("GPS_origin_lat", None)
        GPS_origin_lon = rospy.get_param("GPS_origin_lon", None)
        gps_origin_offset_applied = rospy.get_param("gps_origin_offset_applied", None)
        self.GPS_origin_lat = GPS_origin_lat
        self.GPS_origin_lon = GPS_origin_lon
        rospy.loginfo("GPS_origin_lat in odom: %s, GPS_origin_lon: %s", self.GPS_origin_lat, self.GPS_origin_lon)

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
        # either set to zero or remove entirely
        # the odom statement need quat.  That can come from the raw IMU data.
        self.heading_radians_imu = -1.57
        self.heading_radians_wheels = self.heading_radians_imu
        self.prev_yaw = 0.0
        self.angular_vel_z_imu = 0.0
        self.angular_vel_z_wheel = 0.0
        self.imu_calls = 0
        self.imu_skips = 0
        # the initial quat is calculated based on the pre-defined yaw.  Maybe
        # I don't need this or just use what is in the IMU callback
        self.quat = quaternion_from_euler(0.0, 0.0, self.heading_radians_imu)
        self.wheelbase = 1.27
        self.last_time = rospy.Time.now()
        self.prev_time_imu = rospy.Time.now()
        self.last_print_time = rospy.Time.now()
        self.GPS_origin_lat = 40.34534080  
        self.GPS_origin_lon = -80.12894600   
        rospy.Timer(rospy.Duration(60), self.calibrate_lat_lon_origin)
        #origin_lat = 40.34534080; origin_lon = -80.12894600  # represents the starting point of my tractor inside the garage - averaged on 20230904


        self.x = 0.0
        self.y = 0.0        
        self.x_gps = 0.0
        self.y_gps = 0.0
        self.x_wheel = 0.0
        self.y_wheel = 0.0
        self.x_base_link = 0.0
        self.y_base_link = 0.0        
        self.RTK_fix = False
        self.non_RTK_fix = 0
        self.COG = 0
        self.COG_smoothed = 0
        self.COG_deg = 0
        self.prev_lat = 0
        self.current_lat = 0
        self.prev_lon = 0
        self.current_lon = 0
        self.yaw = 0


        rospy.Subscriber('/left_meters_travelled_msg', Float32, self.left_distance_cb)
        rospy.Subscriber('/right_meters_travelled_msg', Float32, self.right_distance_cb)

        rospy.Subscriber('/left_speed', Float32, self.left_speed_cb)
        rospy.Subscriber('/right_speed', Float32, self.right_speed_cb)

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.hdg_from_imu_pub = rospy.Publisher('hdg_from_imu', Float64, queue_size=1)
        self.hdg_from_wheels_pub = rospy.Publisher('hdg_from_wheels', Float64, queue_size=1)
        self.gps_array_pub = rospy.Publisher('gps_array_data', Float64MultiArray, queue_size=1)        
        
    def check_angle_wrap_radians(self, new_angle, old_angle):
        # if new =  3.1 and old is -3.1, old_angle will be set to  3.1 (turning clockwise)
        # if new = -3.1 and old is  3.1, old_angle will be set to -3.1 (turning counter clockwise)
        diff = new_angle - old_angle
        if diff > math.pi:
            old_angle = new_angle
        elif diff < -math.pi:
            old_angle = new_angle
        return old_angle

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
        self.yaw = yaw
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
        # heading_radians_imu can be set to yaw since it is now useable without adjustment
        self.heading_radians_imu = yaw      
        #self.heading_radians_imu = self.heading_radians_imu + delta_yaw

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
        # I won't need this check because I will rely on IMU data
        if self.heading_radians_imu   > PI:
            self.heading_radians_imu = self.heading_radians_imu  - ( 2 * PI)
        elif self.heading_radians_imu  < -PI:
            self.heading_radians_imu  = self.heading_radians_imu + (2 * PI)
        else:
            self.heading_radians_imu = self.heading_radians_imu 

        # I'm going to test building self.quat from GPS COG
        # self.quat = orientation_list
        self.prev_yaw = yaw
        self.prev_time_imu = current_time_imu

    def gps_callback(self, data):
        # Check to see if we are in GPS FIX mode
        if data.status.status != 2:
            self.non_RTK_fix = self.non_RTK_fix  + 1
            self.RTK_fix = False    
        if data.status.status == 2:
            self.prev_lat = self.current_lat
            self.prev_lon = self.current_lon
            self.current_lat = data.latitude
            self.current_lon = data.longitude
            delta_lon = self.current_lon - self.prev_lon
            delta_lat = self.current_lat - self.prev_lat               
            self.COG = math.atan2(delta_lat, delta_lon)
            self.COG_smoothed = self.check_angle_wrap_radians(self.COG, self.COG_smoothed)
            gain = 0.1  # Adjust this value as needed
            self.COG_smoothed = (1 - gain) * self.COG_smoothed + gain * self.COG

            #yaw_being_used = self.COG_smoothed
            yaw_being_used = self.heading_radians_imu


            self.quat = quaternion_from_euler(0.0, 0.0, yaw_being_used)
            self.COG_deg = math.degrees(self.COG) 
            self.x_gps, self.y_gps = gc.ll2xy(data.latitude, data.longitude, self.GPS_origin_lat, self.GPS_origin_lon)
            x_offset = 0.51  # 20 inches in front of the rear axle
            y_offset = -0.03  # 1 inch to the right of the center line
            x_offset_rotated = x_offset * math.cos(yaw_being_used) - y_offset * math.sin(yaw_being_used)
            y_offset_rotated = x_offset * math.sin(yaw_being_used) + y_offset * math.cos(yaw_being_used)
            self.x_base_link = self.x_gps - x_offset_rotated  # base_link x
            self.y_base_link = self.y_gps - y_offset_rotated  # base_link y
            self.RTK_fix = True
            self.non_RTK_fix = 0
            delta_lat = round(delta_lat, 2)
            delta_lon = round(delta_lon, 2)
            self.COG_deg = round(self.COG_deg, 2)
            self.COG = round(self.COG, 2)
            self.yaw = round(self.yaw, 2)
            self.heading_radians_wheels = round(self.heading_radians_wheels, 2) 
            heading_data_array = Float64MultiArray()            
            heading_data_array.data = [delta_lat, delta_lon, self.COG_deg, self.COG, self.yaw, self.heading_radians_wheels, \
                self.COG_smoothed, self.x_base_link, self.y_base_link]
            self.gps_array_pub.publish(heading_data_array)
                      
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
            self.x_base_link = self.x_gps
            self.y_base_link = self.y_gps
            self.x_wheel = self.x_gps
            self.y_wheel = self.y_gps
        else:
            self.x_base_link = self.x_wheel
            self.y_base_link = self.y_wheel
        # Create and publish Odometry message
        # to publish this I need x,y, quat, linear x and angular z
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time_odom
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x_base_link
        odom_msg.pose.pose.position.y = self.y_base_link
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
        t.transform.translation.x = self.x_base_link
        t.transform.translation.y = self.y_base_link
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
            #odom_pub.print_info()
            odom_pub.last_print_time = rospy.Time.now()
        rate.sleep()