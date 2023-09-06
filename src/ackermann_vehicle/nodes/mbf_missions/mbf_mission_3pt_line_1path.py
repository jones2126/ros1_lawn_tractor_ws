#!/usr/bin/env python3
'''
python3 mbf_mission_3pt_line_1path.py
cd /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/missions

run a mission of 3 points
The mission is sent as a path instead of individual goals

'''
import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs
#from nav_msgs.msg import Odometry
#import tf
#import math
from nav_msgs.msg import Path
from geometry_msgs.msg import  PoseStamped

def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()

# main routine
rospy.init_node("navigation_drive_a_box")

mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
rospy.loginfo("waiting for actionlib")
mbf_ac.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to Move Base Flex action server!")
rospy.loginfo("Creating a three point path")

# instantiate a path
path = Path()
path.header.frame_id = "map"
path.header.seq = 0
path.header.stamp = rospy.Time.now()
seq = 0
path_pub = rospy.Publisher('/drive_path', Path, queue_size=10)

# First move
#Position(23.255, 3.126, 0.000), Orientation(0.000, 0.000, 0.741, 0.671) = Angle: 1.669
pose = PoseStamped()
pose.header.frame_id = "map"
pose.header.seq = seq
pose.pose.position.x = 23.3
pose.pose.position.y = 3.1
pose.pose.position.z = 0
pose.pose.orientation.x = 0.000
pose.pose.orientation.y = 0.000
pose.pose.orientation.z = 0.741
pose.pose.orientation.w = 0.671
pose.header.stamp = path.header.stamp
path.poses.append(pose)
seq += 1

# Second move
#Position(22.838, 8.617, 0.000), Orientation(0.000, 0.000, 0.738, 0.675) = Angle: 1.661
pose = PoseStamped()
pose.header.frame_id = "map"
pose.header.seq = seq
pose.pose.position.x = 22.8
pose.pose.position.y = 8.6
pose.pose.position.z = 0
pose.pose.orientation.x = 0.000
pose.pose.orientation.y = 0.000
pose.pose.orientation.z = 0.738
pose.pose.orientation.w = 0.675
pose.header.stamp = path.header.stamp
path.poses.append(pose)
seq += 1

# Third move
# Position(22.145, 13.623, 0.000), Orientation(0.000, 0.000, 0.753, 0.658) = Angle: 1.705
pose = PoseStamped()
pose.header.frame_id = "map"
pose.header.seq = seq
pose.pose.position.x = 22.1
pose.pose.position.y = 13.6
pose.pose.position.z = 0
pose.pose.orientation.x = 0.000
pose.pose.orientation.y = 0.000
pose.pose.orientation.z = 0.753
pose.pose.orientation.w = 0.658
pose.header.stamp = path.header.stamp
path.poses.append(pose)

rospy.loginfo("sending path")
mbf_ac.send_goal(path)
# path_pub.publish(path)  # path_pub is not defined
rospy.on_shutdown(lambda: mbf_ac.cancel_all_goals())