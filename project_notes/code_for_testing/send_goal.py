#!/usr/bin/env python3
# license removed for brevity
'''

DOES NOT WORK FOR MOVE BASE FLEX
[WARN] [1676304748.108219, 5854.991000]: Failed to send goal:
'MoveBaseGoal' object has no attribute 'path'
[ERROR] [1676304748.109830, 5854.993000]: Action server not available!

also mentioned   File "/opt/ros/noetic/lib/python3/dist-packages/actionlib/simple_action_client.py", line 92, in send_goal

'''
import rospy
import actionlib
import mbf_msgs.msg as mbf_msgs
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathGoal, ExePathResult)

def movebase_client():
    client = actionlib.SimpleActionClient("/move_base_flex/exe_path", mbf_msgs.ExePathAction)
    #client = actionlib.SimpleActionClient('/move_base_flex/exe_path',ExePathAction)
    client.wait_for_server()

    goal = mbf_msgs.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 10.5
    goal.target_pose.pose.position.y = 10.5
    goal.target_pose.pose.orientation.w = 1.0
    #path_goal = create_path_goal(get_path_result.path, True, 1.0, 3.14/18.0)
    client.send_goal(goal)
    wait = client.wait_for_result()
'''
    try:
        client.send_goal(goal)
        wait = client.wait_for_result()
    except Exception as e:
        rospy.logwarn('Failed to send goal:\n%s' % str(e))
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
'''        

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")