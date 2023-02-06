#!/usr/bin/env python3
#
# ref: https://github.com/uos/mbf_tutorials/blob/master/beginner/scripts/mb_relay_subscriber.py
# @author Jorge Santos
# License: 3-Clause BSD
# TODO how to credit 

import actionlib
import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from geometry_msgs.msg import PoseStamped


def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
    rospy.logdebug("Relaying move_base_simple/goal pose to mbf")

    mbf_mb_ac.wait_for_result()

    status = mbf_mb_ac.get_state()
    result = mbf_mb_ac.get_result()

    rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)

if __name__ == '__main__':
    rospy.init_node("move_base_relay")
    rospy.loginfo("mb_relay_subscriber running to send move_base_simple/goal to mbf")
    # move base flex ation client relays incoming mb goals to mbf
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)

    rospy.on_shutdown(lambda: mbf_mb_ac.cancel_all_goals())

    rospy.spin()