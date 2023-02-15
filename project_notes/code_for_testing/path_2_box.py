#!/usr/bin/env python3
# originally copied from: 
# 1. move_base tutorials
# 2. https://github.com/Auburn-Automow/au-automow/blob/adbccbaad4955d72dcac201e57c73a72d6067ba4/au_automow_common/automow_actionlib/nodes/qualification


import rospy
import actionlib
from time import sleep
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathGoal, ExePathResult)
from nav_msgs.msg import Odometry
import tf

###  Variables  ###
orig_pos = None

def odom_callback(data):
    """docstring for odom_callback"""
    global orig_pos
    if orig_pos == None:
        orig_pos = data.pose.pose

def movebase_client():

    client = actionlib.SimpleActionClient('/move_base_flex/exe_path',ExePathAction)
    client.wait_for_server()

    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    # Get pose
    while orig_pos == None and not rospy.is_shutdown():
        rospy.loginfo("Collecting initial pose from odom...")
        rospy.sleep(3.0)
    if rospy.is_shutdown():
        return
    
    (r,p,orig_rot) = tf.transformations.euler_from_quaternion([orig_pos.orientation.x, orig_pos.orientation.y, orig_pos.orientation.z, orig_pos.orientation.w])
    orig_rot += 90    

    # goal = MoveBaseGoal()
    goal = ExePathGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = orig_pos.position.x+0.0
    goal.target_pose.pose.position.y = orig_pos.position.y-5.0
    quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(orig_rot-90))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

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