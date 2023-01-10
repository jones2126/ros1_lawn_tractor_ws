#!/usr/bin/env python3

# This simply repackages a (modified)cmd_vel into an ackermann_cmd msg and publishes it.
# Use with this teb_local_planner when "cmd_angle_instead_rotvel" is set to "True".

import rospy
#import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

def cmd_callback(data):
  global ackermann_cmd_topic
  global pub
  
  v = data.linear.x
  steering = data.angular.z

  # rospy.loginfo("cmd_vel: %f, steering: %f", data.angular.z, steering)

  msg = AckermannDrive()
  msg.steering_angle = steering
  msg.speed = v
  
  pub.publish(msg)
  
if __name__ == '__main__': 
  try:
    
    rospy.init_node('mod_cmd_vel_to_ackermann')
        
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDrive, queue_size=1)
    
    rospy.loginfo("Node 'mod_cmd_vel_to_ackermann' started.\nListening to %s, publishing to %s", twist_cmd_topic, ackermann_cmd_topic)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
