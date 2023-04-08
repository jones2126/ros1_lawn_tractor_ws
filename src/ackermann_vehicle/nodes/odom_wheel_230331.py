#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
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
        rospy.Subscriber('/left_meters_travelled_msg', Float32, self.left_distance_cb)
        rospy.Subscriber('/right_meters_travelled_msg', Float32, self.right_distance_cb)
# right_meters_travelled_msg
        # Publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)

    def left_distance_cb(self, msg):
        self.left_distance = msg.data
        rospy.loginfo("left_distance={:.2f}".format(self.left_distance))
                      
    def right_distance_cb(self, msg):
        self.right_distance = msg.data
        rospy.loginfo("right_distance={:.2f}".format(self.right_distance))
                      
    def publish_odom(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        left_delta = self.left_distance - self.last_left_distance
        rospy.loginfo("left_distance={:.2f}, last_left_distance={:.2f}".format(self.left_distance, self.last_left_distance))
        right_delta = self.right_distance - self.last_right_distance
        rospy.loginfo("right_distance={:.2f}, last_right_distance={:.2f}".format(self.right_distance, self.last_right_distance))
        rospy.loginfo("left_delta={:.3f}, right_delta={:.3f}".format(left_delta, right_delta))
        linear_velocity = ((left_delta + right_delta) / 2.0) / (dt + 1e-6)
        angular_velocity = (right_delta - left_delta) / self.wheelbase / (dt + 1e-6)
        delta_x = linear_velocity * dt * cos(self.theta)
        delta_y = linear_velocity * dt * sin(self.theta)
        delta_theta = angular_velocity * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
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

# the start of Addison's code

# he has this statement earlier in his program
# odomNew = Odometry()
# odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)

        global odomNew, odom_data_pub_quat
        q = Quaternion()
        q.setRPY(0, 0, odomNew.pose.pose.orientation.z)
        odom_msg = Odometry()
        odom_msg.header.stamp = odomNew.header.stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = odomNew.pose.pose.position.x
        odom_msg.pose.pose.position.y = odomNew.pose.pose.position.y
        odom_msg.pose.pose.position.z = odomNew.pose.pose.position.z
        odom_msg.pose.pose.orientation.x = q.x
        odom_msg.pose.pose.orientation.y = q.y
        odom_msg.pose.pose.orientation.z = q.z
        odom_msg.pose.pose.orientation.w = q.w
        odom_msg.twist.twist.linear.x = odomNew.twist.twist.linear.x
        odom_msg.twist.twist.linear.y = odomNew.twist.twist.linear.y
        odom_msg.twist.twist.linear.z = odomNew.twist.twist.linear.z
        odom_msg.twist.twist.angular.x = odomNew.twist.twist.angular.x
        odom_msg.twist.twist.angular.y = odomNew.twist.twist.angular.y
        odom_msg.twist.twist.angular.z = odomNew.twist.twist.angular.z

        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                odom_msg.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                odom_msg.pose.covariance[i] += 0.1
            else:
                odom_msg.pose.covariance[i] = 0

        self.odom_pub.publish(odom_msg)

# The end of Addison's code   

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

# another function from Addison
# 
    def update_odom():
        global odomNew, odomOld, distanceLeft, distanceRight
        # Calculate the average distance
        cycleDistance = (distanceRight + distanceLeft) / 2

        # Calculate the number of radians the robot has turned since the last cycle
        cycleAngle = asin((distanceRight - distanceLeft) / WHEEL_BASE)

        # Average angle during the last cycle
        avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

        if avgAngle > PI:
            avgAngle -= 2 * PI
        elif avgAngle < -PI:
            avgAngle += 2 * PI
        else:
            pass

        # Calculate the new pose (x, y, and theta)
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance
        odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

        # Prevent lockup from a single bad cycle
        if isnan(odomNew.pose.pose.position.x) or isnan(odomNew.pose.pose.position.y) or isnan(odomNew.pose.pose.position.z):
            odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
            odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
            odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

        # Make sure theta stays in the correct range
        if odomNew.pose.pose.orientation.z > PI:
            odomNew.pose.pose.orientation.z -= 2 * PI
        elif odomNew.pose.pose.orientation.z < -PI:
            odomNew.pose.pose.orientation.z += 2 * PI

        # Compute the velocity
        odomNew.header.stamp = rospy.Time.now()
        odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())
        odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.to_sec() - odomOld.header.stamp.to_sec())

        # Save the pose data for the next cycle
        odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
        odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
        odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
        odomOld.header.stamp = odomNew.header.stamp

        # Publish the odometry message
        odom_data_pub.publish(odomNew)        

if __name__ == '__main__':
    odom_pub = OdomPublisher()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom_pub.update_odom()
        odom_pub.publish_odom()
        rate.sleep()
       
