#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/gps/testingNewGPScallbackFunction.py
import rospy
import rosbag
import math
import numpy as np
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler
import geonav_transform.geonav_conversions as gc
from geopy import distance

class TestOdomPublisher:
    def __init__(self):
        self.prev_lat = 0
        self.prev_lon = 0
        self.current_lat = 0
        self.current_lon = 0
        self.last_gps_time = rospy.Time.now()
        self.GPS_origin_lat = 40.485509842
        self.GPS_origin_lon = -80.332308247
        self.max_allowed_speed = 2.5
        self.prev_positions = []
        self.prev_headings = []
        self.prev_speeds = []
        self.COG = 0
        self.COG_smoothed = 0
        self.heading_radians_imu = 0
        self.linear_velocity_from_wheels = 0
        self.gps_array_pub = rospy.Publisher('gps_array_data', Float64MultiArray, queue_size=1)
        #self.yaw = 0.0
        self.yaw_being_used = 0.0

    # Add all the methods we created earlier (is_first_gps_data, handle_first_gps_data, etc.)
    # ...

    def check_angle_wrap_radians(self, new_angle, old_angle):
        diff = new_angle - old_angle
        if diff > math.pi:
            old_angle = new_angle
        elif diff < -math.pi:
            old_angle = new_angle
        return old_angle

    def calculate_new_position(self, start_position, heading, speed, time_delta):
        # speed is already in m/s, time_delta is in seconds
        
        # Calculate distance traveled in meters
        distance_m = speed * time_delta
        
        # Use geopy's distance method to calculate the new position
        d = distance.distance(meters=distance_m)
        return d.destination(point=start_position, bearing=heading)

    def calculate_gps_speed(self, current_time):
        dt = (current_time - self.last_gps_time).to_sec()
        if dt < 0.001:
            rospy.logwarn(f"Too small time difference: {dt}. Skipping this update.")
            return None, dt
        dx, dy = gc.ll2xy(self.current_lat, self.current_lon, self.prev_lat, self.prev_lon)
        distance = math.sqrt(dx*dx + dy*dy)
        return distance / dt, dt

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

    def update_rolling_averages(self, current_speed):
        self.prev_positions.append((self.current_lat, self.current_lon))
        self.prev_headings.append(self.COG)
        self.prev_speeds.append(current_speed)
        
        self.prev_positions = self.prev_positions[-3:]
        self.prev_headings = self.prev_headings[-3:]
        self.prev_speeds = self.prev_speeds[-3:]

    def update_course_over_ground(self):
        delta_lon = self.current_lon - self.prev_lon
        delta_lat = self.current_lat - self.prev_lat
        if self.linear_velocity_from_wheels > 0.1:
            self.COG = math.atan2(delta_lat, delta_lon)
            self.COG = round(self.COG, 2)
        self.COG_smoothed = self.check_angle_wrap_radians(self.COG, self.COG_smoothed)
        gain = 0.1
        self.COG_smoothed = (1 - gain) * self.COG_smoothed + gain * self.COG

        # Update yaw to be the same as COG for this test
        #self.yaw = self.COG_smoothed

        return delta_lat, delta_lon

    def update_position_and_orientation(self, data):
        # yaw_being_used = self.heading_radians_imu
        self.yaw_being_used = self.COG_smoothed   # change this when you have IMU heading data available
        self.quat = quaternion_from_euler(0.0, 0.0, self.yaw_being_used)
        self.COG_deg = math.degrees(self.COG)
        self.x_gps, self.y_gps = gc.ll2xy(data.latitude, data.longitude, self.GPS_origin_lat, self.GPS_origin_lon)
        x_offset = 0.51
        y_offset = -0.03
        x_offset_rotated = x_offset * math.cos(self.yaw_being_used) - y_offset * math.sin(self.yaw_being_used)
        y_offset_rotated = x_offset * math.sin(self.yaw_being_used) + y_offset * math.cos(self.yaw_being_used)
        self.x_base_link = self.x_gps - x_offset_rotated
        self.y_base_link = self.y_gps - y_offset_rotated    

    def publish_gps_data(self, delta_lat, delta_lon):
        self.RTK_fix = True
        self.non_RTK_fix = 0
        delta_lat = round(delta_lat, 2)
        delta_lon = round(delta_lon, 2)
        self.COG_deg = round(self.COG_deg, 2)
        self.COG = round(self.COG, 2)
        self.yaw_being_used = round(self.yaw_being_used, 2)
        self.heading_radians_wheels = round(self.heading_radians_wheels, 2)
        heading_data_array = Float64MultiArray()
        heading_data_array.data = [delta_lat, delta_lon, self.COG_deg, self.COG, self.yaw, self.heading_radians_wheels,
                                   self.COG_smoothed, self.x_base_link, self.y_base_link]
        self.gps_array_pub.publish(heading_data_array)


    def gps_callback(self, data):
        current_time = rospy.Time.now()
        self.current_lat, self.current_lon = data.latitude, data.longitude

        if self.prev_lat == 0 and self.prev_lon == 0:
            self.prev_lat = self.current_lat
            self.prev_lon = self.current_lon
            self.last_gps_time = current_time
            rospy.loginfo("Received first valid GPS data.")
            return

        current_speed_gps, time_delta = self.calculate_gps_speed(current_time)
        if current_speed_gps is None:
            rospy.loginfo("current_speed_gps was None")
            return

        current_speed_gps = self.handle_abnormal_speed(current_speed_gps, time_delta)
        self.update_rolling_averages(current_speed_gps)
        self.gps_linear_velocity = current_speed_gps
        self.last_gps_time = current_time

        delta_lat, delta_lon = self.update_course_over_ground()


        self.update_position_and_orientation(data)
        self.publish_gps_data(delta_lat, delta_lon)

        self.prev_lat, self.prev_lon = self.current_lat, self.current_lon

def main():
    rospy.init_node('test_gps_callback')
    
    odom_publisher = TestOdomPublisher()
    
    bag_path = '/home/tractor/bagfiles/2024-08-28-13-03-43.bag'
    bag = rosbag.Bag(bag_path)
    
    for topic, msg, t in bag.read_messages(topics=['/fix']):
        odom_publisher.gps_callback(msg)
        rospy.sleep(0.1)  # Add a small delay to simulate real-time processing
    
    bag.close()

if __name__ == '__main__':
    main()