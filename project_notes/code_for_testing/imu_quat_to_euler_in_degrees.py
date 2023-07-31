#!/usr/bin/env python3
'''
Below is a Python script that subscribes to the /imu/data topic and print the Euler angles in degrees, 
limiting the range between -180 and +180 degrees.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/imu_quat_to_euler_in_degrees.py

'''

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class IMUListener:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        rospy.init_node('imu_listener', anonymous=True)
        rospy.Subscriber("/imu/data", Imu, self.callback)
        rospy.Timer(rospy.Duration(1), self.print_data)  # Set timer to trigger every second
        rospy.spin()

    def callback(self, data):
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = euler_from_quaternion(quaternion)

        self.roll = euler[0] * (180.0/3.14159)
        self.pitch = euler[1] * (180.0/3.14159)
        self.yaw = euler[2] * (180.0/3.14159)

        # Making sure the values are between -180 and 180
        self.roll = (self.roll + 180) % 360 - 180
        self.pitch = (self.pitch + 180) % 360 - 180
        self.yaw = (self.yaw + 180) % 360 - 180

    def print_data(self, event):
        rospy.loginfo("Roll: %f, Pitch: %f, Yaw: %f", self.roll, self.pitch, self.yaw)

if __name__ == '__main__':
    IMUListener()
