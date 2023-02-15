#!/usr/bin/env python3
'''
mbf_mission_3pt_line.py
/home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

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
#from nav_msgs.msg import Odometry
#import tf
#import math

def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()

# main routine
rospy.init_node("navigation_drive_a_box")

mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
rospy.loginfo("waiting for actionlib")
mbf_ac.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to Move Base Flex action server!")

# First move
#Position(23.255, 3.126, 0.000), Orientation(0.000, 0.000, 0.741, 0.671) = Angle: 1.669
goal = mbf_msgs.MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 23.3
goal.target_pose.pose.position.y = 3.1
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0.000
goal.target_pose.pose.orientation.y = 0.000
goal.target_pose.pose.orientation.z = 0.741
goal.target_pose.pose.orientation.w = 0.671
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)

# Second move
#Position(22.838, 8.617, 0.000), Orientation(0.000, 0.000, 0.738, 0.675) = Angle: 1.661
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 22.8
goal.target_pose.pose.position.y = 8.6
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0.000
goal.target_pose.pose.orientation.y = 0.000
goal.target_pose.pose.orientation.z = 0.738
goal.target_pose.pose.orientation.w = 0.675
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)

# Third move
# Position(22.145, 13.623, 0.000), Orientation(0.000, 0.000, 0.753, 0.658) = Angle: 1.705
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 22.1
goal.target_pose.pose.position.y = 13.6
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0.000
goal.target_pose.pose.orientation.y = 0.000
goal.target_pose.pose.orientation.z = 0.753
goal.target_pose.pose.orientation.w = 0.658
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)

rospy.on_shutdown(lambda: mbf_ac.cancel_all_goals())