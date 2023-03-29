#!/usr/bin/env python3
"""
testing_cmd_vel_pub.py

Produces cmd_vel statements based on input entered into a text file in the format:

linear.x, angular.z, duration (seconds)
0.5, 0.0, 10  <---- 0.5 m/s, go straight for 10 seconds
1.5, 1.0, 15  <---- 1.5 m/s, full left? for 15 seconds

An example command to access one element of the table that is loaded: print(mission_commands[2][1])  # row 2, element 1 - remember 0 reference
The .txt file needs at least two entries otherwise you will get the error: IndexError: invalid index to scalar variable.

sleep_time sets the rate at which twist messages will be published.  At 0.1 the measure Hz using rostopic hz equals 9.9 (i.e. close to 10 HZ)

Ref: http://library.isr.ist.utl.pt/docs/roswiki/mini_max(2f)Tutorials(2f)Moving(20)the(20)Base.html 
Ref: https://github.com/rje1974/Proyecto-Pochito/blob/master/software/base/tractor/catkin_ws/DerechaIquierdaDerechaIzquierda.py (Juan Eduardo Riva) 
"""
import numpy as np
import rospy
from geometry_msgs.msg import Twist

mission_commands = []

def load_file():
    global mission_commands
    filename = 'testing_cmd_vel_pub_input.txt'
    mission_commands = np.loadtxt(filename, delimiter=',')

def run_commands():
    rospy.init_node('move') # first thing, init a node!
    p = rospy.Publisher('cmd_vel', Twist, queue_size=2)  # publish to cmd_vel
    global mission_commands
    speed_ref = 0
    turn_ref = 1
    time_ref = 2
    sleep_time = 0.1  
    for row in mission_commands:
        duration = int(row[time_ref])
        iterations = int(duration / sleep_time)
        twist = Twist()  # create a twist message, and insert speed and angle info
        twist.linear.x = row[speed_ref]
        twist.linear.y = 0  
        twist.linear.z = 0  
        twist.angular.x = 0    
        twist.angular.y = 0
        twist.angular.z = row[turn_ref]
        rospy.loginfo("Running command linear.x: %.2f angular.z: %.2f seconds: %d", row[speed_ref], row[turn_ref], int(row[time_ref]))   
        for i in range(iterations):
            p.publish(twist)
            rospy.sleep(sleep_time)
    twist = Twist() # create a new message that will be blank
    rospy.loginfo("Stopping!")
    p.publish(twist)

if __name__ == '__main__':
    load_file()
    run_commands()