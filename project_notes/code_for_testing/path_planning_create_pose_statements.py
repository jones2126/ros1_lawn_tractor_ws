#! /usr/bin/python3
'''
path_planning_create_pose_statements.py
credit: Modified from a path planner tool I believe is Matt Droter wrote
uses as input the text file of waypoints in the x, y, yaw (radians) format.  Yaw is converted to a quaternion and a pose statement is produced
subscribes to "got_path"
publishes "drive_path"
'''

#import rospy
#from nav_msgs.msg import Path
#from geometry_msgs.msg import  PoseStamped
from tf.transformations import quaternion_from_euler
#from std_msgs.msg import Float64

# main routine
content = {}
try:
    with open('/home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing/north_turn_south_waypoints.txt', 'r') as file:
        content = file.readlines()
        content = [x.strip() for x in content]
except:
    print("File failed to load")

for line in content:
    points = line.split()
    #print(points)
    x = float(points[0])
    y = float(points[1])
    yaw = float(points[2])
    quat = quaternion_from_euler(0, 0, yaw)
    print("create_pose(", round(x,2), "," , round(y,2), ",", 0, ",", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4), "),") 