import rospy
import serial
#import struct
from geometry_msgs.msg import Twist

# Open the serial port
ser = serial.Serial('/dev/ttyACM0', 115200)  # Adjust the port and baud rate as per your setup

# ROS callback for receiving Twist messages
def twist_callback(twist_msg):
    velocity_str = ','.join([
        str(twist_msg.linear.x),
        #str(twist_msg.linear.y),
        #str(twist_msg.linear.z),
        #str(twist_msg.angular.x),
        #str(twist_msg.angular.y),
        str(twist_msg.angular.z)
    ])
    #velocity_str += '\n'  # Add a newline character to the end of the string
    ser.write(velocity_str.encode())
    # print("Sending message:", velocity_str)
    print("Sending message:", velocity_str, "(Length: {})".format(len(velocity_str)))

# ROS initialization and subscription
rospy.init_node('serial_comm_node')
rospy.Subscriber('cmd_vel', Twist, twist_callback)

# Main ROS loop
rospy.spin()