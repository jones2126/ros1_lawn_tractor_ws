#!/usr/bin/env python3
import rospy

rospy.init_node('my_node')

x = 123
theta = 0.456

#rospy.loginfo("x=%f, theta=%f", x, theta)
rospy.loginfo("x={:.3f}, theta={:.3f}".format(x, theta))
