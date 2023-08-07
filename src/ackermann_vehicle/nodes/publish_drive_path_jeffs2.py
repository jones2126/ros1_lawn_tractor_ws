#! /usr/bin/python3
# credit Matt Droter path_publisher.py
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import  PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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

    print("publishing path, 1st attempt")  
    path_pub.publish(path)
    rospy.sleep(1)
    print("publishing path, 2nd attempt")  
    path_pub.publish(path)
    print("Done")


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
        #with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/box_turn.txt', 'r') as file:  # this file is drawn from the folder where the program is started
        #with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/generated_points.txt', 'r') as file:  # this file is drawn from the folder where the program is started
        #with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/435_pv_square.txt', 'r') as file:
        with open('/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/PV_435_3turnoverlap_output.txt', 'r') as file:                     
        #with open('7pts.txt', 'r') as file:
            content = file.readlines()
            content = [x.strip() for x in content]
            print(content)
    except:
        rospy.loginfo("File failed to load")

def print_path_callback(msg):
    # Iterate through the poses in the received Path message
    for pose in msg.poses:
        x = pose.pose.position.x
        y = pose.pose.position.y
        # Assuming yaw is represented as a quaternion, convert it to Euler angles
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)

        # Print the x, y, and yaw values of each pose
        print(f"X: {x}, Y: {y}, Yaw: {yaw}")
    print(f"Total number of poses: {len(msg.poses)}")



if __name__ == '__main__':
    print("starting path_publisher.py")
    rospy.loginfo("starting path_publisher.py")
    rospy.Subscriber('/drive_path', Path, print_path_callback)    
    try:
        load_file()  
        path_publisher()         
    except rospy.ROSInterruptException:
        pass

