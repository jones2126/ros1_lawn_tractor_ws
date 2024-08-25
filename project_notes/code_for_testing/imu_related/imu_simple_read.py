#!/usr/bin/env python3
#
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/imu_related/imu_simple_read.py
#

import rospy
from sensor_msgs.msg import Imu
import tf.transformations
import math

def imu_callback(msg):
    # Extract the orientation quaternion
    orientation_q = msg.orientation
    
    # Convert quaternion to Euler angles
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
    
    # Convert yaw to degrees
    heading_deg = math.degrees(yaw)
    
    # Ensure heading is between -180 and 180 degrees
    if heading_deg > 180:
        heading_deg -= 360
    elif heading_deg < -180:
        heading_deg += 360
    
    # Print the heading
    print(f"IMU Heading: {heading_deg:.2f}°")

def main():
    rospy.init_node('imu_heading_printer', anonymous=True)
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass