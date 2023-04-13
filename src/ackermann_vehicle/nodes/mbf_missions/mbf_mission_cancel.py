#!/usr/bin/env python3
'''
python3 mbf_mission_cancel.py
cd /home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

ref: https://github.com/ros/actionlib/blob/indigo-devel/src/actionlib/action_client.py#L559
get_goal_status
get_goal_status_text
'''

import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs

# main routine
rospy.init_node("navigation_drive_a_box")
mbf_ep_ac = actionlib.SimpleActionClient("move_base_flex/exe_path", mbf_msgs.ExePathAction)
mbf_ep_ac.cancel_all_goals()
mbf_ep_ac.cancel_goal()
#print(mbf_ep_ac.get_goal_status())  # did not work
#print(mbf_ep_ac.get_goal_status_text())  # did not work
#print(mbf_ep_ac.GoalStatus())  # did not workGoalStatus
#print(mbf_ep_ac.getState())  # did not workGoalStatus
t=mbf_ep_ac.get_goal_status_text()
print(t)

'''

Since this command takes ~30 seconds to stop cmd_vel statements, maybe try to send cmd_vel to overide
what is already been submitted.

'''