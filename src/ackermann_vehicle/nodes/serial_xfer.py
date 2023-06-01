import rospy
import serial
import threading
import time
from geometry_msgs.msg import Twist

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 115200)  # Adjust the port and baud rate as per your setup

serial_lock = threading.Lock()

# ROS callback for receiving Twist messages
def twist_callback(twist_msg):
    velocity_str = '1,' + ','.join([
        str(twist_msg.linear.x),
        #str(twist_msg.linear.y),
        #str(twist_msg.linear.z),
        #str(twist_msg.angular.x),
        #str(twist_msg.angular.y),
        str(twist_msg.angular.z)
    ])

    with serial_lock:
        ser.write(velocity_str.encode())
    print("Sending message:", velocity_str, "(Length: {})".format(len(velocity_str)))

# This function runs in a separate thread and checks the speed_params every 5 seconds
def check_speed_params():
    while not rospy.is_shutdown():
        speed_params = rospy.get_param('/speed_params', [])  # Second argument is default value

        numbers_str = '2,' + ','.join(map(str, speed_params))

        with serial_lock:
            ser.write(numbers_str.encode())
        print("Sending message:", numbers_str, "(Length: {})".format(len(numbers_str)))
        time.sleep(5)  # sleep for 5 seconds

def read_serial_port():
    while not rospy.is_shutdown():
        # Check if there's anything to read from the serial port
        while ser.in_waiting > 0:
            with serial_lock:
                line = ser.readline().decode().strip() 
            print("Received message:", line)
# ROS initialization and subscription
rospy.init_node('serial_comm_node')

rospy.Subscriber('cmd_vel', Twist, twist_callback)

# Start threads: 1. Check_speed_params function; 2. Read serial port
threading.Thread(target=check_speed_params).start()
threading.Thread(target=read_serial_port).start()

# Main ROS loop
rospy.spin()
