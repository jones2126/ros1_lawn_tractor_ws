#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathGoal, ExePathResult)

def movebase_client():

    client = actionlib.SimpleActionClient('/move_base_flex/exe_path',ExePathAction)
    client.wait_for_server()

    # goal = MoveBaseGoal()
    goal = ExePathGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 10.5
    goal.target_pose.pose.position.y = 10.5
    goal.target_pose.pose.orientation.w = 1.0
    try:
        client.send_goal(goal)
        wait = client.wait_for_result()
    except Exception as e:
        rospy.logwarn('Failed to send goal:\n%s' % str(e))
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")