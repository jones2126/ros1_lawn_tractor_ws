#!/usr/bin/env python3

import rospy
import serial
import threading
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray

# Add a global variable to signal threads to stop
progControlFlag = True

# cmd_vel variables
linear_x = 0.0
angular_z = 0.0
prev_time_twist = 0.0

# speed variables
left_speed = 0.0
right_speed = 0.0

# gps related variables
gps_status = 0
prev_time_gps_fix = 0.0
gpsStatusAge = 0.0

# Add threading locks for serial ports
ser1_lock = threading.Lock()
ser2_lock = threading.Lock()

iteration = 0

def twist_callback(twist_msg):
    global linear_x, angular_z, prev_time_twist
    linear_x = twist_msg.linear.x
    angular_z = twist_msg.angular.z
    prev_time_twist = rospy.get_time()

def left_speed_callback(left_speed_msg):
    global left_speed
    left_speed = left_speed_msg.data

def right_speed_callback(right_speed_msg):
    global right_speed
    right_speed = right_speed_msg.data    

def read_ttgo_main():
    while progControlFlag and not rospy.is_shutdown():
        with ser1_lock:
            try:
                if ser1.in_waiting > 0:
                    line = ser1.readline().decode('utf-8').strip()

                    if line[0] == '3':
                        components = line.split(',')
                        msg = Float64MultiArray()
                        msg.data = [float(x) for x in components[1:]]
                        pub.publish(msg)
            except serial.SerialException:
                print("****************************************")
                print("read_ttgo_main: SerialException")                
                ser1.close()
                time.sleep(1)
                ser1.open()
            else:
                time.sleep(0.1)

def write_USB2TTL():
    global left_speed, right_speed, linear_x, angular_z, gps_status, gpsStatusAge, iteration
    while progControlFlag and not rospy.is_shutdown():
        with ser2_lock:
            try:
                current_time = rospy.get_time()
                age_of_cmd_vel = current_time - prev_time_twist
                if age_of_cmd_vel > 2.0:
                    linear_x = 0
                    angular_z = 0

                if gpsStatusAge > 1:
                    gps_status = 9

                velocity_str = '1,' + ','.join([
                    str(linear_x),
                    str(angular_z),
                    "{:.4f}".format(left_speed),
                    "{:.4f}".format(right_speed),
                    str(gps_status), 
                    str(gpsStatusAge)])
                ser2.write((velocity_str + '\n').encode())
                print("write_USB2TTL: writing record type 1") 
            except serial.SerialException:
                print("****************************************")
                print("write_USB2TTL: SerialException")                    
                ser2.close()
                time.sleep(1)
                ser2.open()
            else:
                iteration = iteration + 1
                print("write_USB2TTL: sleeping", iteration)
                time.sleep(1)

def shutdown():
    global progControlFlag
    progControlFlag = False
    time.sleep(1)
    ser1.close()
    ser2.close()

# def check_speed_params():
#     while progControlFlag and not rospy.is_shutdown():
#         with ser2_lock:
#             try:
#                 speed_params = rospy.get_param('/speed_params', [])
#                 numbers_str = '2,' + ','.join(map(str, speed_params))
#                 ser2.write((numbers_str + '\n').encode())
#             except serial.SerialException:
#                 print("****************************************")
#                 print("check_speed_params: SerialException")                  
#                 ser2.close()
#                 time.sleep(1)
#                 ser2.open()
#             else:
#                 time.sleep(20)

def fix_callback(msg):
    global prev_time_gps_fix, gpsStatusAge, gps_status
    gps_status = msg.status.status
    gpsStatusAge = (rospy.get_time() - prev_time_gps_fix)
    prev_time_gps_fix = rospy.get_time()

# Initializing the node
rospy.init_node('ROS2portXfer')

rospy.Subscriber('cmd_vel', Twist, twist_callback)
rospy.Subscriber("left_speed", Float32, left_speed_callback)
rospy.Subscriber("right_speed", Float32, right_speed_callback)
rospy.Subscriber("fix", NavSatFix, fix_callback)
pub = rospy.Publisher('my_array_topic', Float64MultiArray, queue_size=10)

# Open the serial ports
try:
    ser1 = serial.Serial('/dev/ttgo_main', 115200)
except Exception as e:
    print("Failed to open serial port /dev/ttgo_main", e)    

try:
    ser2 = serial.Serial('/dev/USB2TTL', 115200)
except Exception as e:
    print("Failed to open serial port /dev/USB2TTL", e)    

rospy.on_shutdown(shutdown)

threading.Thread(target=read_ttgo_main).start()
threading.Thread(target=write_USB2TTL).start()
#threading.Thread(target=check_speed_params).start()

rospy.spin()