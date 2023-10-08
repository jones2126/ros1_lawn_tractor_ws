
# Begin of extracted and modified code

import math
from geometry_msgs.msg import Twist

k = 0.1  # look forward gain
lookahead_distance = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
wheelbase = 2.9  # [m] wheel base of vehicle

#show_animation = True

# Global variable to keep track of the last waypoint that the robot was tracking
current_waypoint_index = 0
#complexity_sw = 0  # used to disinguish whether to adjust lookahead based on speed
threshold = .3

class Robot:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
    #def __init__(self, x=30.175, y=-96.512, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

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
        # self.threshold = 1.0 # tested on 9/8/23 - a left hand curve had the tractor turning right in the middle
        self.threshold = 1.5
        self.last_time = rospy.Time.now()
        self.prev_time_imu = rospy.Time.now()
        self.last_print_time = rospy.Time.now()

        # Fetch parameters from parameter server
        self.GPS_origin_lat = rospy.get_param("GPS_origin_lat", 40.34534080)
        self.GPS_origin_lon = rospy.get_param("GPS_origin_lon", -80.12894600)
        self.gps_origin_offset_applied = rospy.get_param("gps_origin_offset_applied", 0)
        rospy.Timer(rospy.Duration(30), self.calibrate_lat_lon_origin)

        # target 1.8, 1.7

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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.P1 = []
        self.P2 = []
        self.A = [0,0]
        self.goal_point = []

        rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        self.gps_array_pub = rospy.Publisher('gps_array_data', Float64MultiArray, queue_size=1)  
        self.pp_array_pub = rospy.Publisher('pp_array_data', Float64MultiArray, queue_size=1)   


    def update(state, a, UsteerAngle):

    # change/integrate this to be the GPS callback    

        state.x = state.x + state.v * math.cos(state.yaw) * dt
        state.y = state.y + state.v * math.sin(state.yaw) * dt
        # yaw represents the desired orientation that the robot should have based on the calculated steering angle.
        state.yaw = state.yaw + state.v / wheelbase * math.tan(UsteerAngle) * dt
        state.v = state.v + a * dt
        #print(state.x, state.y)
        return state

    def gps_callback(self, data):
        #print("GPS Callback executed. self.A:", self.A)
        # Check to see if we are in GPS FIX mode
      # Check if RTK fixed is achieved and the offset has not been applied yet
    
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
            self.COG = atan2(delta_lat, delta_lon)
            self.COG_smoothed = self.check_angle_wrap_radians(self.COG, self.COG_smoothed)
            gain = 0.1  # Adjust this value as needed
            self.COG_smoothed = (1 - gain) * self.COG_smoothed + gain * self.COG

            #yaw_being_used = self.COG_smoothed
            yaw_being_used = self.heading_radians_imu


            self.quat = quaternion_from_euler(0.0, 0.0, yaw_being_used)
            self.COG_deg = degrees(self.COG) 
            self.x_gps, self.y_gps = gc.ll2xy(data.latitude, data.longitude, self.GPS_origin_lat, self.GPS_origin_lon)
            x_offset = 0.51  # 20 inches in front of the rear axle
            y_offset = -0.03  # 1 inch to the right of the center line
            x_offset_rotated = x_offset * cos(yaw_being_used) - y_offset * sin(yaw_being_used)
            y_offset_rotated = x_offset * sin(yaw_being_used) + y_offset * cos(yaw_being_used)
            self.x_base_link = self.x_gps - x_offset_rotated  # base_link x
            self.y_base_link = self.y_gps - y_offset_rotated  # base_link y
            self.A = [self.x_base_link, self.y_base_link]
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

    def pure_pursuit_control(state, mission_list_x_coordinates, mission_list_y_coordinates, inputWaypointIndex):
        global complexity_sw
        #print("in pure_pursuit_control")
        #ppcWaypointIndex = calcWaypointIndexOrig(state, mission_list_x_coordinates, mission_list_y_coordinates)
        ppcWaypointIndex = calcWaypointIndex(state, mission_list_x_coordinates, mission_list_y_coordinates)

        if inputWaypointIndex >= ppcWaypointIndex:
            ppcWaypointIndex = inputWaypointIndex

        if ppcWaypointIndex < len(mission_list_x_coordinates):
            tx = mission_list_x_coordinates[ppcWaypointIndex]
            ty = mission_list_y_coordinates[ppcWaypointIndex]
        else:
            tx = mission_list_x_coordinates[-1]
            ty = mission_list_y_coordinates[-1]
            ppcWaypointIndex = len(mission_list_x_coordinates) - 1

        angleToGoal = math.atan2(ty - state.y, tx - state.x) - state.yaw
        '''

        '''

        if state.v < 0:  # back
            angleToGoal = math.pi - angleToGoal
        if complexity_sw == 1:
            speed_adjusted_lookahead_distance = k * state.v + lookahead_distance
            print("dynamic lookahead = {:.2f}".format(speed_adjusted_lookahead_distance))
            str_angl = math.atan2(2.0 * wheelbase * math.sin(angleToGoal) / speed_adjusted_lookahead_distance, 1.0)
        else:
            str_angl = math.atan2(2.0 * wheelbase * math.sin(angleToGoal) / lookahead_distance, 1.0)

     #steering_angle = atan((2 * wheelbase * sin(angle_to_goal)) / look_ahead_distance)        

        return str_angl, ppcWaypointIndex

'''
This function determines the index of a point on the given path that the robot should aim for. The purpose of this 
function is to help the robot look ahead on its path rather than simply aiming for the nearest point. 

This function first finds the closest point on the path to the robot and then looks further ahead on the path, 
based on the robot's speed, to determine which point it should target next. 

'''

    def withinThreshold(state, waypoint_x, waypoint_y):
        # checks if robot and current waypoint is within the threshold and sets true/false condition
        global threshold
        distance = math.sqrt((state.x - waypoint_x) ** 2 + (state.y - waypoint_y) ** 2)
        return distance < threshold

    def calcWaypointIndex(state, mission_list_x_coordinates, mission_list_y_coordinates):
        global current_waypoint_index
        
        # Check if the robot is close enough to the current target waypoint
        if withinThreshold(state, mission_list_x_coordinates[current_waypoint_index], mission_list_y_coordinates[current_waypoint_index]):
            # If we haven't reached the end of the path, move to the next waypoint
            if current_waypoint_index < len(mission_list_x_coordinates) - 1:
                current_waypoint_index += 1
            
        return current_waypoint_index   

    def load_mission_data(self):
        '''
        This function assumes the 'mission' is a series of waypoints to achieve
        one after another and the file is in the format:
        x position, space, y position, space, yaw

        This is the way the program 'path_planning_waypoint_generator.py' creates
        paths that follow a dubins path.
        '''
        with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_green_marks2_output.txt', 'r') as file:
            content = file.readlines()

            # Convert content into a list of (x, y) pairs
            coordinates = [(float(line.split()[0]), float(line.split()[1])) for line in content]

            # Initialize the mission list with the robot's current position and the first coordinate from the file
            #lmdMission = [((self.x_base_link, self.y_base_link), coordinates[0])]
            lmdMission = []

            # Add pairs of consecutive coordinates to the mission list
            for i in range(len(coordinates) - 1):
                lmdMission.append((coordinates[i], coordinates[i+1]))
        print(type(lmdMission), len(lmdMission), lmdMission)
        return lmdMission



    def runMission():
        # 1. Read waypoints
        waypoints = read_waypoints_from_csv('path_to_waypoints.csv')
        cx, cy = zip(*waypoints)  # Extract x and y coordinates separately

        # 2. Initialization
        state = State(x=cx[0], y=cy[0])  # Assuming starting at the first waypoint
        
        # Assuming target_speed and other necessary parameters are initialized elsewhere

        # Create ROS publisher and subscriber (if using ROS)
        # cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # rospy.Subscriber('/gps/fix', NavSatFix, state.update_from_gps)

        # 3. Main Control Loop
        while lastWaypointIndex > waypointIndex:
            ai = PIDControl(target_speed, state.v)
            #print("ai =", ai)
            steerAngle, waypointIndex = pure_pursuit_control(state, mission_list_x_coordinates, mission_list_y_coordinates, waypointIndex)
            #print("steer angle = {:.2f}".format(steerAngle))
            #distance_to_last_waypoint = math.sqrt((state.x - mission_list_x_coordinates[-1]) ** 2 + (state.y - mission_list_y_coordinates[-1]) ** 2)
            #print(f"Current waypointIndex: {waypointIndex}, Distance to last waypoint: {distance_to_last_waypoint:.2f}")
            

if __name__ == '__main__':
    steer_robot = Robot()
    mission = steer_robot.load_mission_data()
    steer_robot.runMission()
