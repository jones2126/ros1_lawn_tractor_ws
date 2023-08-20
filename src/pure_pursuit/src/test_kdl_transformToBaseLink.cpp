/*
Used to understand KDL better and more specifically transformToBaseLink
ref: https://wiki.ros.org/kdl, https://orocos.org/kdl/examples

to run:
window 1: roscore
window 2: rosrun pure_pursuit test_kdl_transformToBaseLink

*/
#include <kdl/frames.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <iostream>

// Function to transform a pose into the robot frame (base_link)
KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& tf) {
    // Convert geometry_msgs::Pose to KDL::Frame
    KDL::Frame poseFrame(
        KDL::Rotation::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));

    // Convert geometry_msgs::Transform to KDL::Frame
    KDL::Frame tfFrame(
        KDL::Rotation::Quaternion(tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
        KDL::Vector(tf.translation.x, tf.translation.y, tf.translation.z));

    // Apply the transformation
    return tfFrame * poseFrame;
}

int main(int argc, char** argv) {
    // Sample pose
    geometry_msgs::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    pose.orientation.w = 1.0; // Identity quaternion

    // Sample transform
    geometry_msgs::Transform tf;
    tf.translation.x = 2.0;
    tf.translation.y = 3.0;
    tf.translation.z = 4.0;
    tf.rotation.w = 1.0; // Identity quaternion

    // Call the function
    KDL::Frame transformedPose = transformToBaseLink(pose, tf);

    // Print the result
    KDL::Rotation rotation = transformedPose.M;
    KDL::Vector translation = transformedPose.p;
    double rx, ry, rz, rw;
    rotation.GetQuaternion(rx, ry, rz, rw);

    std::cout << "Transformed Pose (Rotation): " << rx << ", " << ry << ", " << rz << ", " << rw << std::endl;
    std::cout << "Transformed Pose (Translation): " << translation.x() << ", " << translation.y() << ", " << translation.z() << std::endl;

 
    return 0;
}
