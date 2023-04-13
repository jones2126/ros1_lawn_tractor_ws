#!/usr/bin/env python3
'''
mbf_exec_mission.py
credit: https://uos.github.io/mbf_docs/tutorials/beginner/path_planning/

This sends goals to the robot to follow the path using move_base_flex

'''

import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs
import geometry_msgs.msg as geometry_msgs

def create_pose(x, y, z, xx, yy, zz, ww):
    pose = geometry_msgs.PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = xx
    pose.pose.orientation.y = yy
    pose.pose.orientation.z = zz
    pose.pose.orientation.w = ww
    return pose

def create_path_goal(path, tolerance_from_action, dist_tolerance, angle_tolerance):
    goal = mbf_msgs.ExePathGoal()   # ref: https://github.com/magazino/move_base_flex/blob/master/mbf_abstract_nav/src/move_base_action.cpp#L543
    goal.path = path
    goal.tolerance_from_action = tolerance_from_action
    goal.dist_tolerance = dist_tolerance
    goal.angle_tolerance = angle_tolerance
    return goal

def exe_path(path_goal):
    mbf_ep_ac.send_goal(path_goal)
    mbf_ep_ac.wait_for_result()
    return mbf_ep_ac.get_result()

def get_plan(pose):
    path_goal = mbf_msgs.GetPathGoal(target_pose=pose, tolerance=0.5)
    mbf_gp_ac.send_goal(path_goal)
    mbf_gp_ac.wait_for_result()
    return mbf_gp_ac.get_result()

# main function
rospy.init_node("move_base_flex_exec_path")

# the points below represent driving a Dubins path circle that is 5m in diameter.  The "step" is 2m.
dubins_5m = []
hay_cutting = []
mission_steps = [
    create_pose(23.1,  2.8, 0, 0.0, 0.0, 0.7068,  0.7074),
    create_pose(23.1,  7.8, 0, 0.0, 0.0, 0.8660,  0.5000),
    create_pose(22.8,  9.0, 0, 0.0, 0.0, 0.9659,  0.2588),
    create_pose(21.8, 10.0, 0, 0.0, 0.0, 1.0000,  0.0000),
    create_pose(20.6, 10.3, 0, 0.0, 0.0, 0.9659, -0.2588),
    create_pose(19.4, 10.0, 0, 0.0, 0.0, 0.8660, -0.5000),
    create_pose(18.4,  9.0, 0, 0.0, 0.0, 0.7071, -0.7071),
    create_pose(18.1,  7.8, 0, 0.0, 0.0, 0.5000, -0.8660),
    create_pose(18.1,  7.8, 0, 0.0, 0.0, 0.5000, -0.8660),
    create_pose(18.1,  6.8, 0, 0.0, 0.0, 0.7071, -0.7071),
    create_pose(18.1,  5.8, 0, 0.0, 0.0, 0.7071, -0.7071),
    create_pose(18.1,  4.8, 0, 0.0, 0.0, 0.7071, -0.7071),
    create_pose(18.1,  3.8, 0, 0.0, 0.0, 0.7071, -0.7071),
    create_pose(18.1,  2.8, 0, 0.0, 0.0, 0.7071, -0.7071)
    ]

# move_base_flex exe path client
mbf_ep_ac = actionlib.SimpleActionClient("move_base_flex/exe_path", mbf_msgs.ExePathAction)
mbf_ep_ac.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to Move Base Flex ExePath server!")
# move base flex get path client
mbf_gp_ac = actionlib.SimpleActionClient("move_base_flex/get_path", mbf_msgs.GetPathAction)
mbf_gp_ac.wait_for_server(rospy.Duration(10))
rospy.on_shutdown(lambda: mbf_ep_ac.cancel_all_goals())
#for target_pose in target_poses:
for target_pose in mission_steps:    
    rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.pose.position.x, target_pose.pose.position.y)
    get_path_result = get_plan(target_pose)
    if get_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
        rospy.loginfo("Unable to complete get_plan: %s", get_path_result.outcome)
    else:  # get_path_result.outcome is successful
        path_goal = create_path_goal(get_path_result.path, True, 1.0, 3.14/18.0)
        exe_path_result = exe_path(path_goal)
        if exe_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete exe_path: %s", exe_path_result )