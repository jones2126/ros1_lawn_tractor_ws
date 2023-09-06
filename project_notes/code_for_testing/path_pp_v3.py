#!/usr/bin/env python3
'''
This program was written to help me decompose the steps needed to calculate
a steering angle for my robot.  The key approach is using the 'pure pursuit' process
that culminates in the statement:

steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)

My robot is an Ackermann steering style robot.  In field testing the steering angle
will be positve for a left turn and negative for a right turn.

I took an approach that only the desired path and current position are used as input
along with constants for wheelbase and look ahead distance.

I then used the result to plot key data elements to somewhat verify the approach.

P1 and P2 is the path to follow with P1 being the starting position.
'A' is the current position

Al Jones

'''

# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/path_plot_pure_pusuit_simple.py


import matplotlib.pyplot as plt
from math import sqrt, acos, atan, sin, cos
import numpy as np

# constants
look_ahead_distance = 2
wheelbase = 1.27


class Robot:
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
        self.GPS_origin_lat = 40.3452899
        self.GPS_origin_lon = -80.1289039

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


        #rospy.Subscriber('/left_meters_travelled_msg', Float32, self.left_distance_cb)
        #rospy.Subscriber('/right_meters_travelled_msg', Float32, self.right_distance_cb)

        #rospy.Subscriber('/left_speed', Float32, self.left_speed_cb)
        #rospy.Subscriber('/right_speed', Float32, self.right_speed_cb)

        #rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        #self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        #self.hdg_from_imu_pub = rospy.Publisher('hdg_from_imu', Float64, queue_size=1)
        #self.hdg_from_wheels_pub = rospy.Publisher('hdg_from_wheels', Float64, queue_size=1)
        self.gps_array_pub = rospy.Publisher('gps_array_data', Float64MultiArray, queue_size=1)        

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
            heading_data_array.data = [delta_lat, delta_lon, self.COG_deg, self.COG, self.yaw, self.heading_radians_wheels, self.COG_smoothed]
            self.gps_array_pub.publish(heading_data_array)

    # Function to calculate the angle between two 2D points (aka vectors)
    def angle_between(v1, v2):
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        magnitude_v1 = sqrt(v1[0]**2 + v1[1]**2)
        magnitude_v2 = sqrt(v2[0]**2 + v2[1]**2)
        return acos(dot_product / (magnitude_v1 * magnitude_v2))  

    def calculate_steer_angle():
        global P1, P2, A  # inputs
        global closest_point, goal_point, circle_center, radius, angle_to_goal, \
        		off_track_error, steering_angle   # outputs
        # Calculating the closest point (projecting the current position onto the path)

        if P2[0] - P1[0] == 0:  # Check if the path is a vertical line
            closest_point_x = P1[0]
            closest_point_y = A[1]
            closest_point_y = max(min(closest_point_y, P2[1]), P1[1])  # Ensure the closest point lies within the path segment
        elif P2[1] - P1[1] == 0:  # Check if the path is a horizontal line
            closest_point_x = A[0]
            closest_point_y = P1[1]
            closest_point_x = max(min(closest_point_x, P2[0]), P1[0])  # Ensure the closest point lies within the path segment
        else:
            m_path = (P2[1] - P1[1]) / (P2[0] - P1[0])      # Slope of the path line
            b_path = P1[1] - m_path * P1[0]                 # y-intercept of the path line
            m_perp = -1 / m_path                            # Slope of the perpendicular line
            b_perp = A[1] - m_perp * A[0]                   # y-intercept of the perpendicular line
            closest_point_x = (b_perp - b_path) / (m_path - m_perp)  # Solving for the intersection (closest point)
            closest_point_y = m_path * closest_point_x + b_path

        closest_point = (closest_point_x, closest_point_y)

        # Calculating the goal point (B) by moving up the path by the look-ahead distance
        dir_vector = (P2[0] - P1[0], P2[1] - P1[1])  # Direction vector of the path
        dir_magnitude = sqrt(dir_vector[0]**2 + dir_vector[1]**2)  # Magnitude of the direction vector
        unit_dir_vector = (dir_vector[0] / dir_magnitude, dir_vector[1] / dir_magnitude)  # Unit direction vector
        # Calculating the goal point (B) by moving along the path by the look-ahead distance
        goal_point_x = closest_point_x + unit_dir_vector[0] * look_ahead_distance
        goal_point_y = closest_point_y + unit_dir_vector[1] * look_ahead_distance
        # Ensure the goal point lies within the path segment
        goal_point_x = max(min(goal_point_x, max(P1[0], P2[0])), min(P1[0], P2[0]))
        goal_point_y = max(min(goal_point_y, max(P1[1], P2[1])), min(P1[1], P2[1]))
        goal_point = (goal_point_x, goal_point_y)

        # Calculating the circle center (C) that intesects (A) & (B)
        # Midpoint between A and B
        midpoint_x = (A[0] + goal_point_x) / 2
        midpoint_y = (A[1] + goal_point_y) / 2
        # Slope of the line AB
        slope_AB = (goal_point_y - A[1]) / (goal_point_x - A[0]) if goal_point_x != A[0] else float('inf')
        # Slope of the perpendicular bisector
        slope_perp_bisector = -1 / slope_AB if slope_AB != 0 else float('inf')
        # Distance from the midpoint to the circle center
        dist_to_center_squared = look_ahead_distance**2 - ((midpoint_x - A[0])**2 + (midpoint_y - A[1])**2) / 4
        if dist_to_center_squared < 0:
            print("Error: Cannot find a suitable circle center. Try increasing the look-ahead distance or correcting the initial position.")
            # in the real world I would have to have a more elegant way to handle this
        else:
            dist_to_center = sqrt(dist_to_center_squared)

        # Determine the direction to move to get the correct turn direction
        # If the vehicle is to the right of the path, we want a left turn, so we move in the opposite direction of the slope
        if A[0] > P1[0]:
            dist_to_center = -dist_to_center

        # Circle center coordinates
        circle_center_x = midpoint_x + dist_to_center * sqrt(1 / (1 + slope_perp_bisector**2))
        circle_center_y = midpoint_y + slope_perp_bisector * (circle_center_x - midpoint_x)
        circle_center = (circle_center_x, circle_center_y)
        radius = sqrt((circle_center_x - A[0])**2 + (circle_center_y - A[1])**2)  # Calculate the radius to plot the circle

        # Calculate the angle to the goal
        # Calculate the vector from the current position (A) to the goal point (B) and robot
        vector_to_goal = (goal_point[0] - A[0], goal_point[1] - A[1])
        vector_to_robot = (A[0] - closest_point_x, A[1] - closest_point_y)
        # Cross product between the direction vector of the path and the vector_to_robot
        cross_product = dir_vector[0] * vector_to_robot[1] - dir_vector[1] * vector_to_robot[0]

        # Calculate the vector representing the direction of the path
        dir_vector_path = (P2[0] - P1[0], P2[1] - P1[1])
        # Calculate the angle between these two vectors
        angle_to_goal = angle_between(vector_to_goal, dir_vector_path)

        # Calculate the off-track error
        off_track_error = sqrt((closest_point[0] - A[0])**2 + (closest_point[1] - A[1])**2)

        # Determine the sign of the angle based on the relative position of the robot to the path
        # If the robot is to the right of the path, we want a left turn, so we negate the angle
        if cross_product > 0:  # Robot is to the right of the path
            angle_to_goal = -angle_to_goal
            off_track_error = -off_track_error  # Not sure if this is a convention or not

        # Calculate the steering angle using the pure pursuit formula
        steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)

    def load_mission_data(self):
        '''
        This function assumes the 'mission' is a series of waypoints to achieve
        one after another and the file is in the format:
        x position, space, y position, space, yaw

        This is the way the program 'path_planning_waypoint_generator.py' creates
        paths that follow a dubins path.
        '''
        with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_3turnoverlap_output_trimmed.txt', 'r') as file:
            content = file.readlines()

            # Convert content into a list of (x, y) pairs
            coordinates = [(float(line.split()[0]), float(line.split()[1])) for line in content]

            # Initialize the mission list with the robot's current position and the first coordinate from the file
            mission = [(self.x_base_link, self.y_base_link, coordinates[0])]

            # Add pairs of consecutive coordinates to the mission list
            for i in range(len(coordinates) - 1):
                mission.append((coordinates[i], coordinates[i+1]))
        print(type(mission), len(mission), mission)
        return mission


    def close_enough_to_waypoint(state, waypoint_x, waypoint_y):
        global threshold
        """Check if the robot is close enough to the given waypoint."""
        distance = math.sqrt((state.x - waypoint_x) ** 2 + (state.y - waypoint_y) ** 2)
        return distance < threshold

if __name__ == '__main__':
    steer_angle = Robot()
    rate = rospy.Rate(10)
    global P1, P2, A  # inputs

    # Load the mission data
    mission = load_mission_data()

    # Iterate over the waypoints in the mission
    for index, waypoints in enumerate(mission):
        # Extract the P1 and P2 values
        P1, P2 = waypoints

        # Set the A value
        A = get_current_position()  # Assuming you have this function defined elsewhere

        # Publish the steer angle at a rate of 10 Hz until you're close enough to the waypoint
        while not rospy.is_shutdown():
            # If close enough to the waypoint or at the final waypoint, break out of the loop
            if close_enough_to_waypoint(A, P2[0], P2[1]):
                if index == len(mission) - 1:
                    print("Reached the final waypoint!")
                break

            current_time_main = rospy.Time.now()
            steer_angle.publish_steer_angle()
            rate.sleep()
         