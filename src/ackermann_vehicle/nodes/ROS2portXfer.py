#!/usr/bin/env python3

import rospy
import serial
import threading
import time
from std_msgs.msg import Float32, Float64MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

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

# Global variables for storing the strings
velocity_str = ""
numbers_str = ""
iteration = 0

# Serial status
serial_status = [0.0, 0.0]

def twist_callback(twist_msg):
    global linear_x, angular_z, prev_time_twist
    linear_x = twist_msg.linear.x
    angular_z = twist_msg.angular.z
    prev_time_twist = rospy.get_time()

def left_speed_callback(left_speed_msg):
    global left_speed
    #left_speed = left_speed_msg.data
    left_speed = round(left_speed_msg.data[3], 2)  # speed is the 3rd element in the array

def right_speed_callback(right_speed_msg):
    global right_speed
    #right_speed = right_speed_msg.data
    right_speed = round(right_speed_msg.data[3], 2)  # speed is the 3rd element in the array

def fix_callback(msg):
    global prev_time_gps_fix, gpsStatusAge, gps_status
    gps_status = msg.status.status
    gpsStatusAge = (rospy.get_time() - prev_time_gps_fix)
    prev_time_gps_fix = rospy.get_time()    

def read_ttgo_main():
    while progControlFlag and not rospy.is_shutdown():
        with ser1_lock:
            try:
                if ser1.in_waiting > 0:
                    line = ser1.readline().decode('utf-8').strip()

                    if line[0] == '3':
                        components = line.split(',')
                        msg = Float64MultiArray()
                        msg.data = [float(x) for x in components[1:] if x.strip() != '']
                        pub.publish(msg)
            except serial.SerialException:
                rospy.logerr("read_ttgo_main: SerialException")
                ser1.close()
                time.sleep(1)
                ser1.open()
            else:
                time.sleep(0.1)

def create_record_type_1():
    global left_speed, right_speed, linear_x, angular_z, gps_status, gpsStatusAge, iteration, prev_time_twist, velocity_str
    while progControlFlag and not rospy.is_shutdown():
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
        time.sleep(.25)
 
def create_record_type_2():
    global numbers_str
    while progControlFlag and not rospy.is_shutdown():
        speed_params = rospy.get_param('/speed_params', [])
        numbers_str = '2,' + ','.join(map(str, speed_params))
        ser2.write((numbers_str + '\n').encode())
        time.sleep(20)

def write_to_USB2TTL():
    global velocity_str, numbers_str, iteration
    while progControlFlag and not rospy.is_shutdown():
        with ser2_lock:
            try:
                ser2.write((velocity_str + '\n').encode())
                ser2.write((numbers_str + '\n').encode())
            except serial.SerialException:
                rospy.logerr("write_to_serial: SerialException %f", rospy.get_time())
                ser2.close()
                time.sleep(1)
                ser2.open()
            else:
                iteration = iteration + 1
                time.sleep(.5)

def publish_serial_status(event=None):
    global serial_status
    status_msg = Float64MultiArray()
    status_msg.data = serial_status
    serial_status_pub.publish(status_msg)

def shutdown():
    global progControlFlag
    progControlFlag = False
    time.sleep(1)
    ser1.close()
    ser2.close()

# Initializing the node
rospy.init_node('ROS2portXfer')
rospy.loginfo("Starting_ROS2portXfer")

rospy.Subscriber('cmd_vel', Twist, twist_callback)
#rospy.Subscriber("left_speed", Float32, left_speed_callback)
#rospy.Subscriber("right_speed", Float32, right_speed_callback)
rospy.Subscriber('wheel_data_left', Float32MultiArray, left_speed_callback)
rospy.Subscriber('wheel_data_right', Float32MultiArray, right_speed_callback)

rospy.Subscriber("fix", NavSatFix, fix_callback)
pub = rospy.Publisher('my_array_topic', Float64MultiArray, queue_size=10)
serial_status_pub = rospy.Publisher('serial_status', Float64MultiArray, queue_size=10)

# Open the serial ports
try:
    ser1 = serial.Serial('/dev/ttgo_main', 115200)
    rospy.loginfo("Opened serial port /dev/ttgo_main")
    serial_status[0] = 1.0
except Exception as e:
    rospy.logerr("Failed to open serial port /dev/ttgo_main: %s", str(e))

try:
    ser2 = serial.Serial('/dev/USB2TTL', 115200)
    rospy.loginfo("Opened serial port /dev/USB2TTL")
    serial_status[1] = 1.0
except Exception as e:
    rospy.logerr("Failed to open serial port /dev/USB2TTL: %s", str(e))

rospy.on_shutdown(shutdown)

threading.Thread(target=read_ttgo_main).start()
threading.Thread(target=create_record_type_1).start()
threading.Thread(target=create_record_type_2).start()
threading.Thread(target=write_to_USB2TTL).start()

# Publish serial status every second
rospy.Timer(rospy.Duration(1), publish_serial_status)

rospy.spin()