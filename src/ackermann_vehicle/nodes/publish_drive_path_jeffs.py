#! /usr/bin/python3
# credit Matt Droter path_publisher.py
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import  PoseStamped
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float64

got_path = False

def path_callback(msg):
    global got_path
    if msg.data == -1.0:
        got_path = True

def path_publisher():
    rospy.init_node('path_publisher')
    rospy.Subscriber('got_path', Float64, path_callback)
    path_pub = rospy.Publisher('/drive_path', Path, queue_size=10)

    path = Path()
    
    path.header.frame_id = "map"
    path.header.seq = 0
    path.header.stamp = rospy.Time.now()
    
    seq = 0

    for line in content:
        print(line)
        pose = PoseStamped()
        
        points = line.split()
        print(points)
        x = float(points[0])
        y = float(points[1])
        yaw = float(points[2])

        quat = quaternion_from_euler(0, 0, yaw)

        pose.header.frame_id = "map"
        pose.header.seq = seq
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.header.stamp = path.header.stamp
        
        path.poses.append(pose)

        seq += 1

    print("publishing path")  
    path_pub.publish(path)
    rospy.sleep(1)
    print("publishing path")  
    path_pub.publish(path)
    print("Done")  
    rospy.sleep(1)
    quit()

    '''
    while not rospy.is_shutdown():
        if (got_path is False):
            print("publishing path")  
            path_pub.publish(path)
        else:
            break

        rospy.sleep(10)
    '''

content = {}
def load_file():
    print("In load_file")
    # TODO: Get filename from rosparam
    # File format each waypoint on its own line:  
    #  x y yaw
    #  x y yaw
    rospy.loginfo("starting load_file")
    global content
    try:
        with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/box_turn.txt', 'r') as file:  # this file is drawn from the folder where the program is started
        #with open('7pts.txt', 'r') as file:
            content = file.readlines()
            content = [x.strip() for x in content]
            print(content)
    except:
        rospy.loginfo("File failed to load")


if __name__ == '__main__':
    print("starting path_publisher.py")
    rospy.loginfo("starting path_publisher.py")
    try:
        load_file()
        path_publisher()
    except rospy.ROSInterruptException:
        pass

