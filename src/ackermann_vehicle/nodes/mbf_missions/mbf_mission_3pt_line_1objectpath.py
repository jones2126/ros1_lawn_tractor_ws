#!/usr/bin/env python3
'''
python3 mbf_mission_3pt_line_1objectpath.py
cd /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

run a mission of 3 points
The mission is sent as a path instead of individual goals

credit: https://uos.github.io/mbf_docs/tutorials/advanced/smach/

'''

import rospy
import smach
import smach_ros
import threading

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction


def create_pose(x, y, z, xx, yy, zz, ww):
    pose = PoseStamped()
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


def iterate_target_poses():
    target_poses = [   
        create_pose(23.3, 3.1, 0, 0, 0, 0.741, 0.671),
        create_pose(22.8, 8.6, 0, 0, 0, 0.738, 0.675),
        create_pose(22.1, 13.6, 0, 0, 0, 0.753, 0.658)
    ]

    for target_pose in target_poses:
        yield target_pose   # results in an object 

def create_path_goal(path, tolerance_from_action, dist_tolerance, angle_tolerance):
    goal = mbf_msgs.ExePathGoal()
    goal.path = path
    goal.tolerance_from_action = tolerance_from_action
    goal.dist_tolerance = dist_tolerance
    goal.angle_tolerance = angle_tolerance
    return goal

def main():
    rospy.init_node('mbf_state_machine')

    target_poses = iterate_target_poses()

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.target_pose = None
    sm.userdata.path = None
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = False
    sm.userdata.error_status = None

    with sm:
        # path callback
        def get_path_callback(userdata, goal):
            try:
                goal.target_pose = next(target_poses)
            except StopIteration:
                rospy.logwarn("Reached last target pose")
                rospy.signal_shutdown("Last goal reached. Shutting down")

        # Get path
        smach.StateMachine.add(
            'GET_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/get_path',
                GetPathAction,
                goal_cb=get_path_callback,
                goal_slots=['target_pose'],
                result_slots=['path']
            ),
            transitions={
                'succeeded': 'EXE_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

        def path_callback(userdata, goal):
            target_pose = goal.path.poses[-1].pose
            rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.position.x, target_pose.position.y)

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_cb=path_callback,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

    # Execute SMACH plan
    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted 
    smach_thread.join()

if __name__=="__main__":
    main()