#!/usr/bin/env python3

# ackermannstamped2ackermann.py
# Subscribe to AckermannDriveStamp message and convert to AckermannDrive message

import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

class MainClass():
    def __init__(self):

        self.sub_acker_in = rospy.Subscriber('acker_in', AckermannDriveStamped, self.acker_callback)
        self.pub_acker_out = rospy.Publisher('acker_out', AckermannDrive, queue_size=1)
        rospy.spin()

    def acker_callback(self, msg):
        # Create a new non-stamped message
        msg_out = AckermannDrive()
        # Just copy the drive section from the stamped message to a new non-stamped message
        # The header just gets ignored
        msg_out = msg.drive
        self.pub_acker_out.publish(msg_out)

if __name__ == '__main__':
    rospy.loginfo("ackermannstamp2ackermann.py")
    rospy.init_node('ackermannstamp2ackermann')
    node_object = MainClass()

