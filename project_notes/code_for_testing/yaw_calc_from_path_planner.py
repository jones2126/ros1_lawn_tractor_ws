#! /usr/bin/python3

#import rospy
#from nav_msgs.msg import Path
#from geometry_msgs.msg import  PoseStamped
from tf.transformations import quaternion_from_euler
#from std_msgs.msg import Float64


# main routine
input_file_waypoints = "/home/al/ros1_lawn_tractor_ws/project_notes/code_for_testing/5m_circle_waypoints.txt"

content = {}
try:
    with open(input_file_waypoints, 'r') as file:
        content = file.readlines()
        content = [x.strip() for x in content]
except:
    print("Issue with file load")

for line in content:
    #print(line)
    points = line.split()
    #print(points)
    x = float(points[0])
    y = float(points[1])
    yaw = float(points[2])
    quat = quaternion_from_euler(0, 0, yaw)
    print(round(x,2), "," , round(y,2), ",", round(yaw,2), ",", round(quat[0],4), ",", round(quat[1],4), ",", round(quat[2],4), ",", round(quat[3],4), "),")