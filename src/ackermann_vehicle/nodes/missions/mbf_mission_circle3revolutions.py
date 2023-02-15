#!/usr/bin/env python3
'''
mbf_mission_circle5revolutions.py
/home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

This sends goals to the robot to follow a circular path using move_base_flex for 5 revolutions

credit: https://uos.github.io/mbf_docs/tutorials/beginner/path_planning/

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
create_pose(19.9 ,15.03 ,0 ,0 ,0 ,-0.6492 ,0.7606),
create_pose(19.9 ,15.03 ,0 ,0 ,0 ,-0.6492 ,0.7606),
create_pose(19.9 ,15.03 ,0 ,0 ,0 ,-0.6492 ,0.7606),
create_pose(20.17 ,14.23 ,0 ,0 ,0 ,-0.4957 ,0.8685),
create_pose(20.88 ,13.55 ,0 ,0 ,0 ,-0.237 ,0.9715),
create_pose(21.85 ,13.37 ,0 ,0 ,0 ,0.0503 ,0.9987),
create_pose(22.76 ,13.74 ,0 ,0 ,0 ,0.3262 ,0.9453),
#create_pose(23.35 ,14.53 ,0 ,0 ,0 ,0.5682 ,0.8229),
create_pose(23.42 ,15.5 ,0 ,0 ,0 ,0.7797 ,0.6262),
create_pose(22.94 ,16.36 ,0 ,0 ,0 ,0.9262 ,0.3771),
create_pose(22.07 ,16.81 ,0 ,0 ,0 ,0.9956 ,0.0933),
create_pose(21.08 ,16.7 ,0 ,0 ,0 ,0.9796 ,-0.2009),
#create_pose(20.31 ,16.05 ,0 ,0 ,0 ,0.8773 ,-0.48),
create_pose(20.05 ,15.08 ,0 ,0 ,0 ,0.6929 ,-0.7211),
create_pose(20.37 ,14.15 ,0 ,0 ,0 ,0.4524 ,-0.8918),
create_pose(21.15 ,13.55 ,0 ,0 ,0 ,0.1797 ,-0.9837),
create_pose(22.13 ,13.49 ,0 ,0 ,0 ,-0.1128 ,-0.9936),
create_pose(22.98 ,13.97 ,0 ,0 ,0 ,-0.393 ,-0.9196),
create_pose(23.44 ,14.84 ,0 ,0 ,0 ,-0.6259 ,-0.7799),
create_pose(23.39 ,15.82 ,0 ,0 ,0 ,-0.8122 ,-0.5833),
create_pose(22.84 ,16.64 ,0 ,0 ,0 ,-0.9383 ,-0.3457),
create_pose(21.95 ,17.05 ,0 ,0 ,0 ,-0.9968 ,-0.08),
create_pose(20.97 ,16.94 ,0 ,0 ,0 ,-0.9823 ,0.1872),
create_pose(20.18 ,16.35 ,0 ,0 ,0 ,-0.8961 ,0.4438),
create_pose(19.82 ,15.43 ,0 ,0 ,0 ,-0.7438 ,0.6684),
create_pose(19.98 ,14.46 ,0 ,0 ,0 ,-0.5406 ,0.8413),
#create_pose(20.61 ,13.71 ,0 ,0 ,0 ,-0.2877 ,0.9577),
create_pose(21.55 ,13.43 ,0 ,0 ,0 ,0.0015 ,1),
create_pose(22.49 ,13.71 ,0 ,0 ,0 ,0.2863 ,0.9581),
create_pose(23.14 ,14.47 ,0 ,0 ,0 ,0.5437 ,0.8393),
# create_pose(23.25 ,15.47 ,0 ,0 ,0 ,0.7702 ,0.6378),
create_pose(22.8 ,16.34 ,0 ,0 ,0 ,0.9209 ,0.3899),
create_pose(21.94 ,16.82 ,0 ,0 ,0 ,0.9939 ,0.1099),
#create_pose(20.96 ,16.75 ,0 ,0 ,0 ,0.9833 ,-0.1817),
create_pose(20.19 ,16.15 ,0 ,0 ,0 ,0.8923 ,-0.4514),
create_pose(19.84 ,15.23 ,0 ,0 ,0 ,0.7385 ,-0.6743),
create_pose(20.01 ,14.26 ,0 ,0 ,0 ,0.5318 ,-0.8468),
#create_pose(20.66 ,13.52 ,0 ,0 ,0 ,0.2861 ,-0.9582),
create_pose(21.6 ,13.22 ,0 ,0 ,0 ,0.0188 ,-0.9998),
create_pose(22.56 ,13.45 ,0 ,0 ,0 ,-0.2475 ,-0.9689),
create_pose(23.27 ,14.14 ,0 ,0 ,0 ,-0.5012 ,-0.8653),
create_pose(23.51 ,15.09 ,0 ,0 ,0 ,-0.7132 ,-0.7009),
create_pose(23.23 ,16.04 ,0 ,0 ,0 ,-0.8732 ,-0.4873),
create_pose(22.51 ,16.7 ,0 ,0 ,0 ,-0.9752 ,-0.2211),
#create_pose(21.54 ,16.85 ,0 ,0 ,0 ,-0.9975 ,0.0705),
create_pose(20.64 ,16.44 ,0 ,0 ,0 ,-0.9382 ,0.346),
create_pose(20.1 ,15.62 ,0 ,0 ,0 ,-0.8106 ,0.5856),
create_pose(20.05 ,14.64 ,0 ,0 ,0 ,-0.6261 ,0.7797),
create_pose(20.51 ,13.76 ,0 ,0 ,0 ,-0.3948 ,0.9188),
create_pose(21.36 ,13.27 ,0 ,0 ,0 ,-0.1144 ,0.9934),
create_pose(22.34 ,13.34 ,0 ,0 ,0 ,0.1762 ,0.9844),
#create_pose(23.12 ,13.92 ,0 ,0 ,0 ,0.4401 ,0.898),
create_pose(23.49 ,14.83 ,0 ,0 ,0 ,0.6662 ,0.7458),
create_pose(23.33 ,15.81 ,0 ,0 ,0 ,0.8407 ,0.5415),
#create_pose(22.7 ,16.56 ,0 ,0 ,0 ,0.9558 ,0.2941),
create_pose(21.77 ,16.86 ,0 ,0 ,0 ,0.9999 ,0.012),
create_pose(20.82 ,16.61 ,0 ,0 ,0 ,0.9627 ,-0.2705),
create_pose(20.17 ,15.88 ,0 ,0 ,0 ,0.8444 ,-0.5358),
create_pose(20.02 ,14.91 ,0 ,0 ,0 ,0.6541 ,-0.7564),
create_pose(20.42 ,14.02 ,0 ,0 ,0 ,0.4234 ,-0.9059),
create_pose(21.24 ,13.47 ,0 ,0 ,0 ,0.1638 ,-0.9865),
#create_pose(22.22 ,13.41 ,0 ,0 ,0 ,-0.1038 ,-0.9946),
create_pose(23.1 ,13.86 ,0 ,0 ,0 ,-0.3668 ,-0.9303),
create_pose(23.6 ,14.71 ,0 ,0 ,0 ,-0.6188 ,-0.7856),
create_pose(23.54 ,15.69 ,0 ,0 ,0 ,-0.8187 ,-0.5743),
#create_pose(22.97 ,16.47 ,0 ,0 ,0 ,-0.9443 ,-0.3292)
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