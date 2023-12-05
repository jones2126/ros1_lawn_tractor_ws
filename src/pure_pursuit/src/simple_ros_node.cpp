#include <ros/ros.h>

int main(int argc, char **argv) {
    // Initialize the ROS system and become a node.
    std::cout << "Hello from simple_ros_node - std::cout..." << std::endl;
    ros::init(argc, argv, "simple_ros_node");
    ros::NodeHandle nh;

    // Print an info message.
    ROS_INFO("Hello from simple_ros_node - ROS_INFO");

    // Spin, waiting for messages to arrive.
    ros::spin();

    return 0;
}