#!/usr/bin/env python3
'''
mbf_mission_onepoint.py
/home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

send a one point goal to actionlib using move base flex

The pose data was generated by issuing a 2D Nav goal in RIVS and capturing the information
Position(23.255, 3.126, 0.000), Orientation(0.000, 0.000, 0.741, 0.671) = Angle: 1.669

started with: https://github.com/uos/mbf_tutorials/blob/master/beginner/scripts/mbf_goal_client.py

'''

import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs


def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()

# main routine
rospy.init_node("move_base_flex_simple_goal")
mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
mbf_ac.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to Move Base Flex action server!")
goal = mbf_msgs.MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()

goal.target_pose.pose.position.x = 23.3
goal.target_pose.pose.position.y = 3.1
goal.target_pose.pose.position.z = 0
goal.target_pose.pose.orientation.x = 0
goal.target_pose.pose.orientation.y = 0
goal.target_pose.pose.orientation.z = 0.741
goal.target_pose.pose.orientation.w = 0.671
print("loop: goal", goal)
rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
result = move(goal)
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)
rospy.on_shutdown(lambda: mbf_ac.cancel_all_goals())