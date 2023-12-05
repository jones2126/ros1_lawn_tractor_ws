#! /usr/bin/env python3
#
# credit: https://github.com/uos/mbf_tutorials/blob/master/beginner/scripts/mb_relay_client.py
#
import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from actionlib_msgs.msg import GoalStatus

def create_goal(x, y, z, xx, yy, zz, ww):
    goal = mb_msgs.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = xx
    goal.target_pose.pose.orientation.y = yy
    goal.target_pose.pose.orientation.z = zz
    goal.target_pose.pose.orientation.w = ww
    return goal

def move(goal):
    rospy.loginfo("In move: x %s y %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
    client.send_goal(goal)
    rospy.loginfo("Goal sent, waiting")
    client.wait_for_result()
    rospy.loginfo("Done waiting")
    return client.get_state() == GoalStatus.SUCCEEDED


def drive_circle():
    goals = [   create_goal(3.0, 15.0, 0, 0, 0, 0.027, 1.0), 
                create_goal(25.0, 15.0, 0, 0, 0, -0.681, 0.732), 
                create_goal(25.0, 0.0, 0, 0, 0, 1.0, -0.0), 
                create_goal(3.0, 0.0, 0, 0, 0, 0.731, 0.683)

    ]

    for goal in goals:
        rospy.loginfo("Attempting to drive to %s %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        if not move(goal):
            return False

    return True

if __name__ == '__main__':
    try:
        rospy.init_node('mb_relay_client')
        
        client = actionlib.SimpleActionClient('move_base', mb_msgs.MoveBaseAction)
        client.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Connected to SimpleActionServer 'move_base'")

        result = drive_circle()
        rospy.loginfo("Drove circle with result: %s", result)
        
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")