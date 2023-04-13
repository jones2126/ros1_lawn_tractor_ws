#!/usr/bin/env python3
'''
python3 mbf_mission_straight_thenUturn.py
cd /home/al/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

This sends goals to the robot to follow a path using move_base_flex.  The path (i.e. mission)
is to drive a Northerly line make a U turn and return

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
create_pose(21.89,  3.49, 0, 0.0, 0.0,  0.7410, 0.6710),
create_pose(21.89,  4.49, 0, 0.0, 0.0,  0.7410, 0.6710),

create_pose( 21.89 , 12.49 , 0 , 0.0 , 0.0 , 0.7413 , 0.6712 ),
create_pose( 21.94 , 13.29 , 0 , 0.0 , 0.0 , 0.6288 , 0.7776 ),
create_pose( 22.22 , 14.03 , 0 , 0.0 , 0.0 , 0.5012 , 0.8653 ),
create_pose( 22.72 , 14.66 , 0 , 0.0 , 0.0 , 0.3663 , 0.9305 ),
create_pose( 23.37 , 15.11 , 0 , 0.0 , 0.0 , 0.2764 , 0.9611 ),
create_pose( 23.98 , 15.63 , 0 , 0.0 , 0.0 , 0.4169 , 0.909 ),
create_pose( 24.4 , 16.31 , 0 , 0.0 , 0.0 , 0.5522 , 0.8337 ),
create_pose( 24.59 , 17.08 , 0 , 0.0 , 0.0 , 0.6743 , 0.7385 ),
create_pose( 24.54 , 17.87 , 0 , 0.0 , 0.0 , 0.7802 , 0.6255 ),
create_pose( 24.25 , 18.62 , 0 , 0.0 , 0.0 , 0.8649 , 0.5019 ),
create_pose( 23.75 , 19.24 , 0 , 0.0 , 0.0 , 0.932 , 0.3624 ),
create_pose( 23.09 , 19.68 , 0 , 0.0 , 0.0 , 0.9768 , 0.2141 ),
create_pose( 22.32 , 19.9 , 0 , 0.0 , 0.0 , 0.9982 , 0.0608 ),
create_pose( 21.53 , 19.87 , 0 , 0.0 , 0.0 , -0.9959 , 0.0907 ),
create_pose( 20.77 , 19.61 , 0 , 0.0 , 0.0 , -0.9699 , 0.2433 ),
create_pose( 20.14 , 19.13 , 0 , 0.0 , 0.0 , -0.9208 , 0.3902 ),
create_pose( 19.68 , 18.48 , 0 , 0.0 , 0.0 , -0.8495 , 0.5276 ),
#create_pose( 19.43 , 17.72 , 0 , 0.0 , 0.0 , -0.7611 , 0.6486 ),
#create_pose( 19.43 , 16.93 , 0 , 0.0 , 0.0 , -0.6518 , 0.7584 ),
create_pose( 19.67 , 16.17 , 0 , 0.0 , 0.0 , -0.5269 , 0.8499 ),
create_pose( 20.12 , 15.51 , 0 , 0.0 , 0.0 , -0.4529 , 0.8916 ),
create_pose( 20.49 , 14.8 , 0 , 0.0 , 0.0 , -0.5851 , 0.811 ),
create_pose( 20.62 , 14.01 , 0 , 0.0 , 0.0 , -0.6997 , 0.7144 ),
create_pose( 20.61 , 13.71 , 0 , 0.0 , 0.0 , -0.7413 , 0.6712 ),

create_pose(20.85, 12.71, 0 ,0.0, 0.0, -0.7410, 0.6710),
create_pose(20.85,  3.71, 0 ,0.0, 0.0, -0.7410, 0.6710)
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
        #path_goal = create_path_goal(get_path_result.path, True, 1.0, 3.14/18.0)
        path_goal = create_path_goal(get_path_result.path, True, 1.0, 3.14/18.0)        
        exe_path_result = exe_path(path_goal)
        if exe_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete exe_path: %s", exe_path_result )