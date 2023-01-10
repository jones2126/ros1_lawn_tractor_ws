#! /usr/bin/env python3

# Modifed from https://www.youtube.com/watch?v=I_5leJK8vhQ
# Slight changes for names
# Added transform publisher using: https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
# Converted to tf2

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

# import tf
# import tf_conversions
import tf2_ros
import geometry_msgs.msg

# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odom_pub')

odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
# odom_broadcaster = tf.TransformBroadcaster()
# odom_broadcaster = tf2_ros.TransformBroadcaster()

rospy.wait_for_service('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom = Odometry()
header = Header()
header.frame_id = 'odom'

model = GetModelStateRequest()
model.model_name = 'ackermann_vehicle'

r = rospy.Rate(20)

while not rospy.is_shutdown():
	result = get_model_srv(model)

	current_time = rospy.Time.now()

	# ===========================
	'''
	odom_broadcaster.sendTransform(
		(result.pose.position.x, result.pose.position.y, 0.0) ,
		(result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
		current_time,
		"base_footprint",
		"odom"
	)
	'''
	# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
	br = tf2_ros.TransformBroadcaster()
	t = geometry_msgs.msg.TransformStamped()

	t.header.stamp = current_time
	t.header.frame_id = "odom"
	t.child_frame_id = "base_footprint"
	t.transform.translation.x = result.pose.position.x
	t.transform.translation.y = result.pose.position.y
	t.transform.translation.z = 0.0
	t.transform.rotation.x = result.pose.orientation.x
	t.transform.rotation.y = result.pose.orientation.y
	t.transform.rotation.z = result.pose.orientation.z
	t.transform.rotation.w = result.pose.orientation.w

	br.sendTransform(t)
	# ===========================

	odom.child_frame_id = 'base_footprint'
	odom.pose.pose = result.pose
	odom.twist.twist = result.twist

	header.stamp = current_time
	odom.header = header

	odom_pub.publish(odom)

	r.sleep()

'''
$ rosservice call /gazebo/get_world_properties
$ rosservice call /gazebo/get_model_state "model_name: 'ackermann_vehicle'"
header: 
  seq: 2
  stamp: 
    secs: 919
    nsecs: 860000000
  frame_id: ''
pose: 
  position: 
    x: 0.2839776058677019
    y: -0.22051119531021324
    z: -2.7961596094083774e-09
  orientation: 
    x: 6.068159409579105e-07
    y: -8.583278410858911e-07
    z: -0.6016347418518831
    w: 0.7987713298533647
twist: 
  linear: 
    x: 0.0004669421045893614
    y: -0.0014777850654544514
    z: 0.0014092595490306364
  angular: 
    x: -0.005591753070711812
    y: -0.007067018603142364
    z: 0.00021976985994102822
success: True
status_message: "GetModelState: got properties"

https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist

$ rossrv show gazebo_msgs/GetModelState
string model_name
string relative_entity_name
---
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
bool success
string status_message

$ rosmsg show Odometry
[nav_msgs/Odometry]:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
'''
