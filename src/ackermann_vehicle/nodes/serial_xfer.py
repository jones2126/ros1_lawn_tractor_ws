#!/usr/bin/env python3
import rospy
import serial
import threading
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

# Open the serial port
try:
    ser2 = serial.Serial('/dev/USB2TTL', 115200)
    print("Successfully opened serial port.")
except Exception as e:
    print("Failed to open serial port:", e)    


left_speed = 0.0
right_speed = 0.0
gps_status = 0
prev_time_gps_fix = 0.0
gpsStatusAge = 0.0

# ROS callback for receiving Twist messages
def twist_callback(twist_msg):
    global left_speed, right_speed
    gpsStatusAge = (rospy.get_time() - prev_time_gps_fix)  
    velocity_str = '1,' + ','.join([
        str(twist_msg.linear.x),
        str(twist_msg.angular.z),
        str(left_speed),
        str(right_speed),
        str(gps_status),     
        str(gpsStatusAge)
    ])

    ser2.write(velocity_str.encode())
    print("Sending message:", velocity_str, "(Length: {})".format(len(velocity_str)))

def left_speed_callback(left_speed_msg):
    global left_speed
    left_speed = left_speed_msg.data

def right_speed_callback(right_speed_msg):
    global right_speed
    right_speed = right_speed_msg.data

# This function runs in a separate thread and checks the speed_params every 5 seconds
def check_speed_params():
    while not rospy.is_shutdown():
        speed_params = rospy.get_param('/speed_params', [])  # Second argument is default value

        numbers_str = '2,' + ','.join(map(str, speed_params))

        ser2.write(numbers_str.encode())
        #print("Sending message:", numbers_str, "(Length: {})".format(len(numbers_str)))
        time.sleep(20)  # sleep for 20 seconds

def fix_callback(msg):
    global prev_time_gps_fix, gpsStatusAge
    gps_status = msg.status.status
    gpsStatusAge = (rospy.get_time() - prev_time_gps_fix)
    prev_time_gps_fix = rospy.get_time()          

# ROS initialization and subscription
rospy.init_node('serial_comm_node')

rospy.Subscriber('cmd_vel', Twist, twist_callback)
rospy.Subscriber("left_speed", Float32, left_speed_callback)
rospy.Subscriber("right_speed", Float32, right_speed_callback)

# Start threads: 1. Check_speed_params function; 2. Read serial port
threading.Thread(target=check_speed_params).start()
print("In seerial_xfer, completed initialization")

# Main ROS loop
rospy.spin()
