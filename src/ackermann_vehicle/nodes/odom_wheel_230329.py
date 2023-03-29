#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster, TransformStamped
from tf.transformations import quaternion_from_euler
from math import cos, sin

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.last_left_distance = 0.0
        self.last_right_distance = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = -1.509
        self.wheelbase = 1.27
        self.last_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber('/left_meters_travelled', Float64, self.left_distance_cb)
        rospy.Subscriber('/right_meters_travelled', Float64, self.right_distance_cb)

        # Publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)

    def left_distance_cb(self, msg):
        self.left_distance = msg.data

    def right_distance_cb(self, msg):
        self.right_distance = msg.data

    def publish_odom(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        # Calculate distance travelled by each wheel
        left_delta = self.left_distance - self.last_left_distance
        right_delta = self.right_distance - self.last_right_distance

        # Calculate linear and angular velocities
        linear_velocity = (left_delta + right_delta) / 2.0 / dt
        angular_velocity = (right_delta - left_delta) / self.wheelbase / dt

        # Calculate change in position and orientation
        delta_x = linear_velocity * dt * cos(self.theta)
        delta_y = linear_velocity * dt * sin(self.theta)
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

        # Broadcast transform between odom and base_footprint frames
        br = TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        br.sendTransform(t)

        self.last_left_distance = self.left_distance
        self.last_right_distance = self.right_distance
        self.last_time = current_time

if __name__ == '__main__':
    odom_pub = OdomPublisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom_pub.publish_odom()
        rate.sleep()
       
