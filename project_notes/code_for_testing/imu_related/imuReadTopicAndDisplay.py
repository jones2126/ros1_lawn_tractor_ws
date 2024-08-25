#!/usr/bin/env python3
# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/imu_related/imuReadTopicAndDisplay.py
'''

The script subscribes to the ROS topic "/imu/data" to receive IMU data. It then processes this data by converting 
the orientation quaternion to Euler angles, focusing on the yaw angle to calculate the heading in degrees. The 
script continuously prints the current heading, providing a simple way to monitor and verify the output of 
my IMU.

Additionally it reads and prints the non-standard magnetometer accuracy data that is stored in a covariance field.

'''


import rospy
from sensor_msgs.msg import Imu
import tf.transformations
import math

def interpret_accuracy(accuracy):
    # New function to interpret the accuracy value
    if accuracy == 0:
        return "Unknown"
    elif accuracy == 1:
        return "Low"
    elif accuracy == 2:
        return "Medium"
    elif accuracy == 3:
        return "High"
    else:
        return f"Unexpected value: {accuracy}"

def imu_callback(msg):
    # Extract the orientation quaternion
    orientation_q = msg.orientation
    # Convert quaternion to Euler angles
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
    # Convert yaw to degrees.  This format is commonly referred to as the "signed angle" or "signed degree" representation where
    # angles or headings will be between -180 and +180 degrees.  The IMU is expected to follow
    # "ENU" (East-North-Up) coordinate system where the x-axis points East is 0 degrees, compass North is 90 degrees, 
    # compass West is 180/-180 degrees and compass South is -90 degrees.
    heading_deg = math.degrees(yaw)
    # Ensure heading is between -180 and 180 degrees
    if heading_deg > 180:
        heading_deg -= 360
    elif heading_deg < -180:
        heading_deg += 360
    
    # Extract magnetometer accuracy from orientation_covariance
    mag_accuracy = msg.orientation_covariance[0]
    accuracy_interpretation = interpret_accuracy(mag_accuracy)

    # Display the heading and magnetometer accuracy
    print(f"Current heading: {heading_deg:.2f}°, Magnetometer Accuracy: {accuracy_interpretation}")

def main():
    # Initialize the ROS node
    rospy.init_node('imu_heading_display', anonymous=True)

    # Create a subscriber for the IMU data
    rospy.Subscriber("/imu/data", Imu, imu_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass