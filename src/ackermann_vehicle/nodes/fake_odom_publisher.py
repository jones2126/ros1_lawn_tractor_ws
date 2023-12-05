# python3 /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/fake_odom_publisher.py
# publishes a fake odom and tf assuming position (0, 0) and heading of 0.05 radians

# Importing necessary libraries for ROS and transformations
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf
from tf.transformations import quaternion_from_euler

# Function to publish odometry data and broadcast transform
def publish_odom():
    # Initialize the ROS node
    rospy.init_node('fake_odom_publisher', anonymous=True)

    # Create a publisher for the /odom topic
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)

    # Create a tf broadcaster
    br = tf.TransformBroadcaster()

    # Set the rate of publishing
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Create an Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        # Set the position (x, y, z)
        odom_msg.pose.pose.position = Point(0, 0, 0)

        # Set the orientation (quaternion)
        # Assuming a heading of 0.05 radians
        q = quaternion_from_euler(0, 0, 0.05)
        odom_msg.pose.pose.orientation = Quaternion(*q)

        # Publish the odometry message
        odom_pub.publish(odom_msg)

        # Broadcast the transform
        br.sendTransform((0, 0, 0), q, rospy.Time.now(), "base_footprint", "odom")

        # Wait until the next iteration
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_odom()
    except rospy.ROSInterruptException:
        pass
