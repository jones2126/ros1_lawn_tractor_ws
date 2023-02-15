#!/usr/bin/env python3
'''
python3 mbf_mission_4pt_box.py
cd /home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

copied from: https://github.com/uos/mbf_tutorials/blob/master/beginner/scripts/mbf_goal_client.py
runs a simple mission that outlines a box starting from the current odom position
first move is to move down/South/-1.57 and end with a heading of 3.14 (mag 270)
second move is to move left/West/3.14 and end with a heading of 1.57 (mag 360/North)
third move is to move up/North/1.57 and end with a heading of 1.57 (mag 90/East)
fourth move is to move right/East/0.0 and end with a heading of -1.57 (mag 180/South) 
'''
import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs
from nav_msgs.msg import Odometry
import tf
import math

def odom_callback(data):
    global orig_pos
    if orig_pos == None:
        orig_pos = data.pose.pose

def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()

# main routine
global orig_pos
orig_pos = None
rospy.init_node("navigation_drive_a_box")

mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
rospy.loginfo("waiting for actionlib")
mbf_ac.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to Move Base Flex action server!")

# Get pose from odom
rospy.Subscriber("/odom", Odometry, odom_callback)
while orig_pos == None and not rospy.is_shutdown():
    rospy.loginfo("Collecting data from odom...")
    rospy.sleep(3.0)

(r,p,orig_rot) = tf.transformations.euler_from_quaternion([orig_pos.orientation.x, orig_pos.orientation.y, orig_pos.orientation.z, orig_pos.orientation.w])

# First move
goal = mbf_msgs.MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = orig_pos.position.x+0.0
goal.target_pose.pose.position.y = orig_pos.position.y-5.0
goal.target_pose.pose.position.z = 0
quat = tf.transformations.quaternion_from_euler(0, 0, 3.14)
goal.target_pose.pose.orientation.x = quat[0]
goal.target_pose.pose.orientation.y = quat[1]
goal.target_pose.pose.orientation.z = quat[2]
goal.target_pose.pose.orientation.w = quat[3]
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)

# Second move
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = orig_pos.position.x-5.0
goal.target_pose.pose.position.y = orig_pos.position.y-5.0
goal.target_pose.pose.position.z = 0
quat = tf.transformations.quaternion_from_euler(0, 0, 1.57)
goal.target_pose.pose.orientation.x = quat[0]
goal.target_pose.pose.orientation.y = quat[1]
goal.target_pose.pose.orientation.z = quat[2]
goal.target_pose.pose.orientation.w = quat[3]
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)

# Third move
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = orig_pos.position.x-5.0
goal.target_pose.pose.position.y = orig_pos.position.y+0.0
goal.target_pose.pose.position.z = 0
quat = tf.transformations.quaternion_from_euler(0, 0, 0)
goal.target_pose.pose.orientation.x = quat[0]
goal.target_pose.pose.orientation.y = quat[1]
goal.target_pose.pose.orientation.z = quat[2]
goal.target_pose.pose.orientation.w = quat[3]
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)

# Fourth move
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = orig_pos.position.x
goal.target_pose.pose.position.y = orig_pos.position.y
goal.target_pose.pose.position.z = 0
quat = tf.transformations.quaternion_from_euler(0, 0, -1.57)
goal.target_pose.pose.orientation.x = quat[0]
goal.target_pose.pose.orientation.y = quat[1]
goal.target_pose.pose.orientation.z = quat[2]
goal.target_pose.pose.orientation.w = quat[3]
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)     

rospy.on_shutdown(lambda: mbf_ac.cancel_all_goals())