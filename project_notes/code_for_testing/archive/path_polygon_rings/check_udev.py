#!/usr/bin/env python
'''
Script that checks whether ports defined in the udev rules are available.

$ python3 ~/ros1_lawn_tractor_ws/src/ackermann_vehicle/nodes/check_udev.py

'''
import rospy
import os
from std_msgs.msg import String

def check_devices():
    device_links = ["ttgo_main", "gps", "odom_left", "odom_right", "rosimu", "USB2TTL"]
    results = {}
    
    for link in device_links:
        path = f"/dev/{link}"
        results[link] = os.path.exists(path)
    
    return results

def publish_device_status():
    rospy.init_node('device_status_publisher', anonymous=True)
    pub = rospy.Publisher('/device_status', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    while not rospy.is_shutdown():
        device_status = check_devices()
        status_message = ", ".join([f"{key}: {'available' if value else 'not available'}" for key, value in device_status.items()])
        rospy.loginfo(status_message)
        pub.publish(status_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_device_status()
    except rospy.ROSInterruptException:
        pass
