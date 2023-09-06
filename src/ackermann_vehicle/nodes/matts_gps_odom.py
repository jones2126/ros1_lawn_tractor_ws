#!/usr/bin/env python3
# gps_odom.py  credit Matt Droter
import rospy
import tf
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Transform, Quaternion, QuaternionStamped, Pose, Point
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference

# Import geonav tranformation module
import sys
sys.path.append('/home/tractor/catkin_ws/src/geonav_transform/src/')
import geonav_transform.geonav_conversions as gc
from imp import reload
reload(gc)


#_GPS_origin_lat = 40.345307365
#_GPS_origin_lon = -80.1288668917

# corner edge of concrete at 435 Pine Valley Dr as you pull up the hill
_GPS_origin_lat = 40.345245345
_GPS_origin_lon = -80.128990477
# garage:  40.345309421806114, -80.12893254116138
# block 1: 40.34528795703812, -80.12892449453484
# block 2: 40.34526547013098, -80.1289177890127
class MainClass():

    def __init__(self):
        self.heading_sub = rospy.Subscriber("heading", QuaternionStamped, self.heading_callback)
        self.fix_sub = rospy.Subscriber("fix", NavSatFix, self.gps_callback)
        # self.odom_pub = rospy.Publisher('test_odom', Odometry, queue_size=1)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.last_heading = ()
        self.last_heading = (0.0, 0.0, 0.0, 1.0)
        self.odom_quat = ()


    def heading_callback(self, data):

        if math.isnan(data.quaternion.x) | math.isnan(data.quaternion.y) | math.isnan(data.quaternion.z) | math.isnan(data.quaternion.w):
            #self.odom_quat = self.last_heading
            self.odom_quat = (0.0,
                0.0,
                0.0,
                1.0)
            self.odom_quat = self.last_heading
        else:
            # If you need to rotate your heading.
            #q_rot = quaternion_from_euler(0, 0, -1.57)
            #q_new = quaternion_multiply(q_rot, self.odom_quat)
            #print q_new

            self.odom_quat = (data.quaternion.x, data.quaternion.y, data.quaternion.z, data.quaternion.w)
            self.last_heading = self.odom_quat



    def gps_callback(self, data):
        """ Takes in the gps LLA data and converts it to local coordinates"""
        _xg = 0.0
        _yg = 0.0

        # Check to see if we are in GPS FIX mode
        if data.status.status != 2:
        	rospy.loginfo("Bad GPS status - data.status.status: %s", data.status.status)   # for debugging
        if data.status.status == 2:
            #Convert from lat/lon to x/y
            _xg, _yg = gc.ll2xy(data.latitude, data.longitude, _GPS_origin_lat, _GPS_origin_lon)

        # Build Odometry Message
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "odom"
        # self.odom_msg.child_frame_id = "base_link"
        self.odom_msg.child_frame_id = "base_footprint"

        # set the position
        self.odom_msg.pose.pose = Pose(Point(_xg, _yg, 0.), Quaternion(*self.odom_quat))
        self.odom_msg.pose.covariance[0] = data.position_covariance[0]  # x
        self.odom_msg.pose.covariance[4] = data.position_covariance[4]  # y
        self.odom_msg.pose.covariance[7] = data.position_covariance[8] # yaw

        # Publish Transform between odom and base_link
        odom_broadcaster = tf.TransformBroadcaster()
        #odom_broadcaster.sendTransform((_xg, _yg, 0.0), self.odom_quat, rospy.Time.now(), "base_link", "odom")
        if data.status.status == 2:
            odom_broadcaster.sendTransform((_xg, _yg, 0.0), self.odom_quat, rospy.Time.now(), "base_footprint", "odom")      
        
    def publish_node(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            self.odom_pub.publish(self.odom_msg)

            rate.sleep()
        rospy.logwarn('GPS odom shutting down.')
        rospy.spin()


if __name__ == '__main__':
    rospy.loginfo("start gps_odom.py")
    rospy.init_node('odom_node')
    node_object = MainClass()
    try:
        node_object.publish_node()
    except rospy.ROSInterruptException:
        pass