#!/usr/bin/env python3
# driving_star_example.py
# credit: https://uos.github.io/mbf_docs/tutorials/beginner/path_planning/
'''
Goal Position(17.3,  9.3, 0.000), Orientation(0.000, 0.000, -0.365, 0.931) = Angle: -0.746
Goal Position(23.9, 16.1, 0.000), Orientation(0.000, 0.000,  0.393, 0.919) = Angle:  0.808
Goal Position(15.9, 21.3, 0.000), Orientation(0.000, 0.000,  0.935, 0.354) = Angle:  2.418
Goal Position(10.6, 15.4, 0.000), Orientation(0.000, 0.000, -0.926, 0.378) = Angle: -2.366
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
    goal = mbf_msgs.ExePathGoal()
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
rospy.init_node("move_base_flex_client")

target_poses = [ 
    create_pose(12.5, 14.3, 0, 0.0, 0.0, 0.8668, 0.4987),
    create_pose(7.5, 14.3, 0, 0.0, 0.0, 1.0, 0.0),
    create_pose(5.0, 10.0, 0, 0.0, 0.0, -0.8668, 0.4987),
    create_pose(7.5, 5.7, 0, 0.0, 0.0, -0.4987, 0.8668),
    create_pose(12.5, 5.7, 0, 0.0, 0.0, 0.0, 1.0),
    create_pose(12.5, 5.7, 0, 0.0, 0.0, 0.4987, 0.8668),
    create_pose(12.5, 14.3, 0, 0.0, 0.0, 0.8668, 0.4987),
    create_pose(7.5, 14.3, 0, 0.0, 0.0, 1.0, 0.0),
    create_pose(5.0, 10.0, 0, 0.0, 0.0, -0.8668, 0.4987),
    create_pose(7.5, 5.7, 0, 0.0, 0.0, -0.4987, 0.8668),
    create_pose(12.5, 5.7, 0, 0.0, 0.0, 0.0, 1.0),
    create_pose(12.5, 5.7, 0, 0.0, 0.0, 0.4987, 0.8668)
]

dubins_5m = [
    create_pose( 0.0 , 9.9 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
    create_pose( 1.2 , 9.9 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
    create_pose( 4.79 , 8.38 , 0 , -0.0 , 0.0 , 0.3894 , -0.9211 ),
    create_pose( 7.53 , 5.54 , 0 , -0.0 , 0.0 , 0.2342 , -0.9722 ),
    create_pose( 11.41 , 5.26 , 0 , 0.0 , 0.0 , 0.1629 , 0.9866 ),
    create_pose( 14.32 , 7.85 , 0 , 0.0 , 0.0 , 0.5342 , 0.8453 ),
    create_pose( 14.49 , 11.74 , 0 , 0.0 , 0.0 , 0.8213 , 0.5706 ),
    create_pose( 11.82 , 14.57 , 0 , 0.0 , 0.0 , 0.9786 , 0.2057 ),
    create_pose( 7.93 , 14.63 , 0 , -0.0 , 0.0 , 0.9815 , -0.1916 ),
    create_pose( 5.07 , 11.93 , 0 , -0.0 , 0.0 , 0.9039 , -0.4278 ),
    create_pose( 1.63 , 10.12 , 0 , -0.0 , 0.0 , 0.9991 , -0.042 ),
    create_pose( 1.2 , 10.1 , 0 , 0.0 , 0.0 , 1.0 , 0.0008 )
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
for target_pose in dubins_5m:    
    rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.pose.position.x, target_pose.pose.position.y)
    get_path_result = get_plan(target_pose)
    if get_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
        rospy.loginfo("Unable to complete get_plan: %s", get_path_result.outcome)
    else:  # get_path_result.outcome is successful
        path_goal = create_path_goal(get_path_result.path, True, 0.5, 3.14/18.0)
        exe_path_result = exe_path(path_goal)
        if exe_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete exe_path: %s", exe_path_result )