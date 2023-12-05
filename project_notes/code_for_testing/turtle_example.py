#!/usr/bin/env python3
#turtle_example.py
# ref: https://uos.github.io/mbf_docs/tutorials/beginner/path_planning/
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


def drive_circle():
    target_poses = [ 
create_pose( 5.0 , 5.0 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
create_pose( 9.0 , 5.0 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
create_pose( 13.0 , 5.0 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
create_pose( 17.0 , 5.0 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
create_pose( 21.0 , 5.0 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
create_pose( 25.0 , 5.0 , 0 , 0.0 , 0.0 , 0.0 , 1.0 ),
create_pose( 28.399447999060488 , 3.188046893607133 , 0 , -0.0 , 0.0 , 0.38600969512590455 , -0.9224947237078411 ),
create_pose( 32.071832094572436 , 2.0817371416613204 , 0 , 0.0 , 0.0 , 0.10351215261005425 , 0.9946281889540598 ),
create_pose( 34.986957245883445 , 4.574200093123885 , 0 , 0.0 , 0.0 , 0.5676906152745418 , 0.8232419846735298 ),
create_pose( 34.4646708326666 , 8.373876805001963 , 0 , 0.0 , 0.0 , 0.8928786164174544 , 0.4502974309769626 ),
create_pose( 30.985160574586256 , 9.987362031501792 , 0 , -0.0 , 0.0 , 0.9994587920309802 , -0.03289563849469746 ),
create_pose( 27.656674538476484 , 8.004016105968214 , 0 , -0.0 , 0.0 , 0.9351456317695885 , -0.3542635281570164 ),
create_pose( 25.0 , 7.0 , 0 , 0.0 , 0.0 , 0.9999996829318346 , 0.0007963267107332633 ),
create_pose( 21.00000000269374 , 7.000003045794576 , 0 , -0.0 , 0.0 , 0.9999999999999678 , -2.5381621398517445e-07 ),
create_pose( 17.000000002694257 , 7.000001015264864 , 0 , -0.0 , 0.0 , 0.9999999999999678 , -2.5381621398517445e-07 ),
create_pose( 13.00000000269477 , 6.999998984735152 , 0 , -0.0 , 0.0 , 0.9999999999999678 , -2.5381621398517445e-07 ),
create_pose( 9.000000002695288 , 6.99999695420544 , 0 , -0.0 , 0.0 , 0.9999999999999678 , -2.5381621398517445e-07 ),
create_pose( 5.000000005389019 , 6.99999999999143 , 0 , 0.0 , 0.0 , 0.9999996829323711 , 0.0007963260371056567 )
]


    for target_pose in target_poses:
        rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.pose.position.x, target_pose.pose.position.y)

        get_path_result = get_plan(target_pose)
        if get_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete plan: %s", result.message)
            return 

        path_goal = create_path_goal(get_path_result.path, True, 0.5, 3.14/18.0)

        exe_path_result = exe_path(path_goal)
        if exe_path_result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete exe: %s", result.message)
            return 


if __name__ == '__main__':
    rospy.init_node("move_base_flex_client")

    # move_base_flex exe path client
    mbf_ep_ac = actionlib.SimpleActionClient("move_base_flex/exe_path", mbf_msgs.ExePathAction)
    mbf_ep_ac.wait_for_server(rospy.Duration(10))
    rospy.loginfo("Connected to Move Base Flex ExePath server!")

    # move base flex get path client
    mbf_gp_ac = actionlib.SimpleActionClient("move_base_flex/get_path", mbf_msgs.GetPathAction)
    mbf_gp_ac.wait_for_server(rospy.Duration(10))

    drive_circle()

    rospy.on_shutdown(lambda: mbf_ep_ac.cancel_all_goals())