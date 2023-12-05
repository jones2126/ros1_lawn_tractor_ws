#!/usr/bin/env python3
# encoding: utf-8
# originally copied from: https://github.com/Auburn-Automow/au-automow/blob/adbccbaad4955d72dcac201e57c73a72d6067ba4/au_automow_common/automow_actionlib/nodes/qualification
###  Imports  ###

# ROS imports
#import roslib; roslib.load_manifest('automow_actionlib')
import rospy
import actionlib
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathGoal, ExePathResult)
from mbf_msgs.msg import (ExePathAction, ExePathGoal)
from time import sleep
import math
import tf

# ROS msg and srv imports
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
# from housekeeping.srv import CutterControl

# Python Libraries
import sys

# Peer Libraries

###  Variables  ###
orig_pos = None

###  Functions  ###

def odom_callback(data):
    """docstring for odom_callback"""
    global orig_pos
    if orig_pos == None:
        orig_pos = data.pose.pose

def main():
    # sleep(10)
    global orig_pos
    orig_pos = None
    rospy.init_node("navigation_drive_a_box")
    client = actionlib.SimpleActionClient('/move_base_flex/exe_path',ExePathAction)
    print("waiting for actionlib") 
    client.wait_for_server()
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    # Get pose
    while orig_pos == None and not rospy.is_shutdown():
        rospy.loginfo("Collecting data from odom...")
        rospy.sleep(3.0)
    if rospy.is_shutdown():
        return
    
    print("z, w", orig_pos.orientation.z, orig_pos.orientation.w)
    (r,p,orig_rot) = tf.transformations.euler_from_quaternion([orig_pos.orientation.x, orig_pos.orientation.y, orig_pos.orientation.z, orig_pos.orientation.w])
    print("original euler:", orig_rot)
    orig_rot += 90
    print("original euler+90:", orig_rot)
    
    # First move
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
        rospy.loginfo("Sending goal: %f, %f, %f"%(destination.target_pose.pose.position.x,destination.target_pose.pose.position.x,orig_rot-90))
        client.send_goal(goal)
        wait = client.wait_for_result()  
    except Exception as e:
        rospy.logwarn('Failed to send goal:\n%s' % str(e))
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
   
    if client.wait_for_result():
        rospy.loginfo("Goal reached")
    else:
        rospy.logwarn("Did not reach goal")
    
    # Second move
    destination.target_pose.header.stamp = rospy.Time.now()
    
    destination.target_pose.pose.position.x = orig_pos.position.x-5.0
    destination.target_pose.pose.position.y = orig_pos.position.y-5.0
    quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(orig_rot-180))
    destination.target_pose.pose.orientation.x = quat[0]
    destination.target_pose.pose.orientation.y = quat[1]
    destination.target_pose.pose.orientation.z = quat[2]
    destination.target_pose.pose.orientation.w = quat[3]
    
    rospy.loginfo("Sending goal: %f, %f, %f"%(destination.target_pose.pose.position.x,destination.target_pose.pose.position.x,orig_rot-180))
    
    client.send_goal(destination)
    
    if client.wait_for_result():
        rospy.loginfo("Goal reached")
    else:
        rospy.logwarn("Did not reach goal")
    
    # Third Move
    destination.target_pose.header.stamp = rospy.Time.now()
    
    destination.target_pose.pose.position.x = orig_pos.position.x-5.0
    destination.target_pose.pose.position.y = orig_pos.position.y+0.0
    quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(orig_rot-270))
    destination.target_pose.pose.orientation.x = quat[0]
    destination.target_pose.pose.orientation.y = quat[1]
    destination.target_pose.pose.orientation.z = quat[2]
    destination.target_pose.pose.orientation.w = quat[3]
    
    rospy.loginfo("Sending goal: %f, %f, %f"%(destination.target_pose.pose.position.x,destination.target_pose.pose.position.x,orig_rot-270))
    
    client.send_goal(destination)
    
    if client.wait_for_result():
        rospy.loginfo("Goal reached")
    else:
        rospy.logwarn("Did not reach goal")
    
    # Final Move
    destination.target_pose.header.stamp = rospy.Time.now()
    
    destination.target_pose.pose.position.x = orig_pos.position.x
    destination.target_pose.pose.position.y = orig_pos.position.y
    quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(orig_rot))
    destination.target_pose.pose.orientation.x = quat[0]
    destination.target_pose.pose.orientation.y = quat[1]
    destination.target_pose.pose.orientation.z = quat[2]
    destination.target_pose.pose.orientation.w = quat[3]
    
    rospy.loginfo("Sending goal: %f, %f, %f"%(destination.target_pose.pose.position.x,destination.target_pose.pose.position.x,orig_rot))
    
    client.send_goal(destination)
    
    if client.wait_for_result():
        rospy.loginfo("Goal reached")
    else:
        rospy.logwarn("Did not reach goal")

###  If Main  ###
if __name__ == '__main__':
    try:
        main()
    except Exception as err:
        rospy.logerr("Unhandled Exception in Qualification Node: \n"+str(err))