#!/usr/bin/env python3
# odom_from_wheel_and_gps.py
import rospy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import cos, sin, pi
import time
import math
import numpy as np  # new line 8/28/24
from geopy import distance  # new line 8/28/24

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
        GPS_origin_lat = rospy.get_param("base_station_lat", None)
        GPS_origin_lon = rospy.get_param("base_station_lon", None)
        # gps_origin_offset_applied = rospy.get_param("gps_origin_offset_applied", None)
        self.GPS_origin_lat = GPS_origin_lat
        self.GPS_origin_lon = GPS_origin_lon
        rospy.loginfo("GPS_origin_lat in odom: %s, GPS_origin_lon: %s", self.GPS_origin_lat, self.GPS_origin_lon)

    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.left_distance = 0.0
        self.right_distance = 0.0
        # self.left_distance_prev = 0.0
        # self.right_distance_prev = 0.0
        self.left_delta = 0.0
        self.right_delta = 0.0
        self.left_speed = 0
        self.right_speed = 0
        # either set to zero or remove entirely
        # the odom statement need quat.  That can come from the raw IMU data.
        #self.heading_radians_imu = -1.57   # 435 Pine Valley, in the garage pointing South
        self.heading_radians_imu = 0.0      # 62 Collins Dr, pointing East
        self.heading_radians_wheels = self.heading_radians_imu
        self.prev_yaw = 0.0
        self.angular_vel_z_imu = 0.0
        # self.angular_vel_z_wheel = 0.0
        self.imu_calls = 0
        self.imu_skips = 0
        # the initial quat is calculated based on the pre-defined yaw.  Maybe
        # I don't need this or just use what is in the IMU callback
        self.quat = quaternion_from_euler(0.0, 0.0, self.heading_radians_imu)
        self.wheelbase = 1.27
        self.last_time = rospy.Time.now()
        self.prev_time_imu = rospy.Time.now()
        self.last_print_time = rospy.Time.now()
        # self.GPS_origin_lat = 40.34534080  #435 Pine Valley
        # self.GPS_origin_lon = -80.12894600 
# <!-- fix position 40.485509842 -80.332308247 357.9  the GPS antennae location -->
        self.GPS_origin_lat = 40.485509842  #62 Collins Dr - antennae location near the fire pit
        self.GPS_origin_lon = -80.332308247

        rospy.Timer(rospy.Duration(60), self.calibrate_lat_lon_origin)
        #origin_lat = 40.34534080; origin_lon = -80.12894600  # represents the starting point of my tractor inside the garage - averaged on 20230904


        # self.x = 0.0
        # self.y = 0.0        
        self.x_gps = 0.0
        self.y_gps = 0.0
        self.x_base_link_gps = 0.0
        self.y_base_link_gps = 0.0
        self.x_base_link_wheel = 0.0
        self.y_base_link_wheel = 0.0
        self.x_base_link = 0.0
        self.y_base_link = 0.0        
        self.RTK_fix = False
        self.gps_status = 0
        self.non_RTK_fix = 0
        self.COG = 0
        self.COG_smoothed = 0
        self.COG_deg = 0
        self.prev_lat = 0
        self.current_lat = 0
        self.prev_lon = 0
        self.current_lon = 0
        self.yaw = 0
        self.linear_velocity_from_wheels = 0.0

        # added on 8/6/24 to calculate speed based on GPS lat, lon changes
        self.last_gps_time = rospy.Time.now()
        self.linear_velocity_from_gps = 0.0
        # end of added code on 8/6/24

        # 8/6/24 added to accomodate new topics containing wheel speed data
        self.left_delta = 0.0 
        self.right_delta = 0.0
        # end of 8/6/24 change

        # Added to filter anomaly lat, lon data using a rolling average
        self.prev_positions = []  # new line 8/28/24
        self.prev_headings = []  # new line 8/28/24
        self.prev_speeds = []  # new line 8/28/24
        self.max_allowed_speed = 2.5  # m/s, new line 8/28/24


        # 8/6/24 changed to accomodate new topics containing wheel speed data
        # rospy.Subscriber('/left_meters_travelled_msg', Float32, self.left_distance_cb)
        # rospy.Subscriber('/right_meters_travelled_msg', Float32, self.right_distance_cb)        
        #rospy.Subscriber('/left_speed', Float32, self.left_speed_cb)
        #rospy.Subscriber('/right_speed', Float32, self.right_speed_cb)
        rospy.Subscriber('wheel_data_left', Float32MultiArray, self.left_wheel_data_cb)
        rospy.Subscriber('wheel_data_right', Float32MultiArray, self.right_wheel_data_cb)
        # end of 8/6/24 change


        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        #rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("fix", NavSatFix, self.gps_callback, queue_size=5)
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

    # 8/6/24 changed to accomodate new topics containing wheel speed data
    # def left_distance_cb(self, msg):
    #     self.left_distance = msg.data
    # def right_distance_cb(self, msg):
    #     self.right_distance = msg.data
    # def left_speed_cb(self, msg):
    #     self.left_speed = msg.data
    # def right_speed_cb(self, msg):
    #     self.right_speed = msg.data       
    def left_wheel_data_cb(self, msg):
        self.left_speed = msg.data[3]
        self.left_delta = msg.data[4]
        self.left_distance += self.left_delta       
    def right_wheel_data_cb(self, msg):
        self.right_speed = msg.data[3]
        self.right_delta = msg.data[4]
        self.right_distance += self.right_delta
    # end of 8/6/24 change

    # 8/28/24 new method to help filter anomaly lat, lon data using a rolling average
    def calculate_new_position(self, start_position, heading, speed, time_delta):
        # new method 8/28/24
        d = distance.distance(meters=speed * time_delta)
        return d.destination(point=start_position, bearing=heading)                      

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


    def calculate_gps_speed(self, current_time):
        dt = (current_time - self.last_gps_time).to_sec()
        if dt < 0.001:
            rospy.logwarn(f"Too small time difference: {dt}. Skipping this update.")
            return None, dt
# 'll2xy' is being called in two places.  Here and 'update_position_and_orientation'
# I could use  self.x_gps, self.y_gps from  function 'update_position_and_orientation' instead of calling ll2xy again
# Or I could put ll2xy at a higher spot and then use self.x_gps, self.y_gps consistently
# remember self.x_gps, self.y_gps are offset by the GPS position in the function 'update_position_and_orientation'
# maybe I need a new variable with the base_link_x and base_link_y position
        dx, dy = gc.ll2xy(self.current_lat, self.current_lon, self.prev_lat, self.prev_lon)
        distance = math.sqrt(dx*dx + dy*dy)
        speed = distance / dt
        # Determine direction based on wheel speeds
        avg_wheel_speed = (self.left_speed + self.right_speed) / 2
        if avg_wheel_speed < 0:
            speed = -speed

        return speed, dt

    def handle_abnormal_speed(self, current_speed, dt):
        if current_speed > self.max_allowed_speed:
            rospy.logwarn(f"Detected abnormal speed: {current_speed} m/s. GPS status: {self.gps_status}")
            if len(self.prev_positions) >= 3:
                avg_heading = np.mean(self.prev_headings[-3:])
                avg_speed = np.mean(self.prev_speeds[-3:])
                new_position = self.calculate_new_position(self.prev_positions[-1], avg_heading, avg_speed, dt)
                self.current_lat, self.current_lon = new_position.latitude, new_position.longitude
                return avg_speed
            else:
                rospy.logwarn("Not enough historical data. Using previous position.")
                self.current_lat, self.current_lon = self.prev_lat, self.prev_lon
                return 0
        return current_speed

    def calculate_new_position(self, start_position, heading, speed, time_delta):
        # speed is already in m/s, time_delta is in seconds
        
        # Calculate distance traveled in meters
        distance_m = speed * time_delta
        
        # Use geopy's distance method to calculate the new position
        d = distance.distance(meters=distance_m)
        return d.destination(point=start_position, bearing=heading)

    def calculate_new_position(self, start_position, heading, speed, time_delta):
        # speed is already in m/s, time_delta is in seconds
        
        # Calculate distance traveled in meters
        distance_m = speed * time_delta
        
        # Use geopy's distance method to calculate the new position
        d = distance.distance(meters=distance_m)
        return d.destination(point=start_position, bearing=heading)

    def check_angle_wrap_radians(self, new_angle, old_angle):
        diff = new_angle - old_angle
        if diff > math.pi:
            old_angle = new_angle
        elif diff < -math.pi:
            old_angle = new_angle
        return old_angle        

    def update_course_over_ground(self):
        delta_lon = self.current_lon - self.prev_lon
        delta_lat = self.current_lat - self.prev_lat
        
        if abs(self.linear_velocity_from_wheels) > 0.05:  # Only update COG if we're moving
            self.COG = math.atan2(delta_lat, delta_lon)
            if self.linear_velocity_from_wheels < 0:
                # Adjust COG for reverse motion
                self.COG += math.pi
                if self.COG > math.pi:
                    self.COG -= 2 * math.pi
            self.COG = round(self.COG, 2)
        
        self.COG_smoothed = self.check_angle_wrap_radians(self.COG, self.COG_smoothed)
        gain = 0.1
        self.COG_smoothed = (1 - gain) * self.COG_smoothed + gain * self.COG

        return delta_lat, delta_lon         
              
    def update_position_and_orientation(self, data):
        self.yaw_being_used = self.heading_radians_imu
        # self.yaw_being_used = self.COG_smoothed   # comment out this line when you have good IMU heading data available
        self.quat = quaternion_from_euler(0.0, 0.0, self.yaw_being_used)
        self.COG_deg = math.degrees(self.COG)
        self.x_gps, self.y_gps = gc.ll2xy(data.latitude, data.longitude, self.GPS_origin_lat, self.GPS_origin_lon)
        x_offset = 0.51
        y_offset = -0.03
        x_offset_rotated = x_offset * math.cos(self.yaw_being_used) - y_offset * math.sin(self.yaw_being_used)
        y_offset_rotated = x_offset * math.sin(self.yaw_being_used) + y_offset * math.cos(self.yaw_being_used)
        #self.self.x_gps = self.x_gps - x_offset_rotated
        self.x_base_link_gps = self.x_gps - x_offset_rotated
        #self.self.y_gps = self.y_gps - y_offset_rotated
        self.y_base_link_gps = self.y_gps - y_offset_rotated

    def publish_gps_data(self, delta_lat, delta_lon):
        #self.RTK_fix = True
        #self.non_RTK_fix = 0
        #delta_lat = round(delta_lat, 8)
        #delta_lon = round(delta_lon, 8)
        self.COG_deg = round(self.COG_deg, 2)
        self.COG = round(self.COG, 2)
        #self.yaw_being_used = round(self.yaw_being_used, 2)
        self.heading_radians_wheels = round(self.heading_radians_wheels, 2)
        heading_data_array = Float64MultiArray()
        heading_data_array.data = [delta_lat, delta_lon, self.COG_deg, self.COG, self.yaw, self.heading_radians_wheels,
                                   self.COG_smoothed, self.x_base_link_gps, self.y_base_link_gps]
        self.gps_array_pub.publish(heading_data_array)

    def update_rolling_averages(self, current_speed):
        self.prev_positions.append((self.current_lat, self.current_lon))
        self.prev_headings.append(self.COG)
        self.prev_speeds.append(current_speed)
        
        # Keep only the last 3 entries
        self.prev_positions = self.prev_positions[-3:]
        self.prev_headings = self.prev_headings[-3:]
        self.prev_speeds = self.prev_speeds[-3:]        

    def gps_callback(self, data):
        current_time = rospy.Time.now()

        # Check if this is the first valid GPS data
        if self.prev_lat == 0 and self.prev_lon == 0:
            self.prev_lat = data.latitude
            self.prev_lon = data.longitude
            self.last_gps_time = current_time
            rospy.loginfo(f"Received first valid GPS data. Latitude: {self.prev_lat:.8f}, Longitude: {self.prev_lon:.8f}")
            return

        self.current_lat = data.latitude
        self.current_lon = data.longitude
        current_speed_gps = None
        time_delta = None            
        self.gps_status = data.status.status
        if data.status.status == 2:
            self.RTK_fix = True
            current_time = rospy.Time.now()  # added on 8/6/24         
            current_speed_gps, time_delta = self.calculate_gps_speed(current_time)
            self.linear_velocity_from_gps = self.handle_abnormal_speed(current_speed_gps, time_delta)
            self.update_rolling_averages(current_speed_gps)
            delta_lat, delta_lon = self.update_course_over_ground()
            self.update_position_and_orientation(data)
        else:
            self.RTK_fix = False 
            self.non_RTK_fix = self.non_RTK_fix  + 1
            delta_lon = self.current_lon - self.prev_lon
            delta_lat = self.current_lat - self.prev_lat
            rospy.logwarn(f"Non-RTK fix received. Status: {data.status.status}. Non-RTK count: {self.non_RTK_fix}")           

        if current_speed_gps is None:
            rospy.loginfo(f"current_speed_gps was None. x_gps: {self.x_gps:.2f}, y_gps: {self.y_gps:.2f}")            
            return

        self.publish_gps_data(delta_lat, delta_lon)  

        self.prev_lat = self.current_lat
        self.prev_lon = self.current_lon

        self.last_gps_time = current_time        

# check yaw being used.  Should be IMU - done
# seems handle_abnormal_speed could be inside 'calculate_gps_speed'
# where is gps_linear_velocity used?  Its used in the odom statement if there is RTK fix.


    def publish_odom(self):
        current_time_odom = rospy.Time.now()
        delta_time_odom = (current_time_odom - self.last_time).to_sec()

        # Calculate distance travelled by each wheel - there will be a rollover event that is not programmed yet
        # self.left_delta = self.left_distance - self.left_distance_prev
        # self.right_delta = self.right_distance - self.right_distance_prev

        # Calculate the distance travelled and speed by the robot using the average distance of the two wheels
        distance = (self.left_delta + self.right_delta) / 2
        self.linear_velocity_from_wheels = (self.left_speed + self.right_speed) / 2
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

        # Normalize heading to [-PI, PI]
        # if self.heading_radians_wheels   > PI:
        #     self.heading_radians_wheels = self.heading_radians_wheels  - ( 2 * PI)
        # elif self.heading_radians_wheels  < -PI:
        #     self.heading_radians_wheels  = self.heading_radians_wheels + (2 * PI)
        # else:
        #     self.heading_radians_wheels = self.heading_radians_wheels 
        # the above code can be condensed to the next statement
        self.heading_radians_wheels = (self.heading_radians_wheels + PI) % (2 * PI) - PI

        # Calculate the change in x and y position of the robot using wheel data
        delta_x = abs(distance) * cos(self.heading_radians_wheels) * np.sign(self.linear_velocity_from_wheels)
        delta_y = abs(distance) * sin(self.heading_radians_wheels) * np.sign(self.linear_velocity_from_wheels)


        self.x_base_link_wheel += delta_x
        self.y_base_link_wheel += delta_y

        # Create and publish Odometry message
        # to publish this I need x,y, quat, linear x and angular z
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time_odom
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"        
        if self.RTK_fix == True:
            odom_msg.pose.pose.position.x = self.x_base_link_gps
            odom_msg.pose.pose.position.y = self.y_base_link_gps
            self.x_base_link_wheel = self.x_base_link_gps  # save location to add delta_x to in case we lose RTK
            self.y_base_link_wheel = self.y_base_link_gps
            odom_msg.twist.twist.linear.x = self.linear_velocity_from_gps  # added on 8/6/24
        else:
            odom_msg.pose.pose.position.x = self.x_base_link_wheel
            odom_msg.pose.pose.position.y = self.y_base_link_wheel            
            odom_msg.twist.twist.linear.x = self.linear_velocity_from_wheels  # added on 8/6/24

        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = self.quat[0]  # 'quat' is defined in function 'update_position_and_orientation'
        odom_msg.pose.pose.orientation.y = self.quat[1]
        odom_msg.pose.pose.orientation.z = self.quat[2]
        odom_msg.pose.pose.orientation.w = self.quat[3]
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_vel_z_imu
        self.odom_pub.publish(odom_msg)

# instead of two publish statements you could publish one array with 2 elements or 3 elements if you include gps yaw
        self.hdg_from_imu_pub.publish(self.heading_radians_imu)
        self.hdg_from_wheels_pub.publish(self.heading_radians_wheels)
    

        # Broadcast transform between odom and base_footprint frames
        br = TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = current_time_odom
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.quat[0]  # 'quat' is defined in function 'update_position_and_orientation'
        t.transform.rotation.y = self.quat[1]
        t.transform.rotation.z = self.quat[2]
        t.transform.rotation.w = self.quat[3]

        br.sendTransform(t)

        # self.left_distance_prev = self.left_distance
        # self.right_distance_prev = self.right_distance
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