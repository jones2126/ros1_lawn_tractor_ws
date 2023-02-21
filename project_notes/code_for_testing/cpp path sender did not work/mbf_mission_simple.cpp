/* 
mbf_mission_simple.cpp

A program to send a path (aka a mission), comprised of a number of pose positions, to move base flex

ref: https://answers.ros.org/question/313661/writing-a-custom-path-using-move-base-flex-exe-path-action/

 */

#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient; // A client of Exe Path Action Server

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_path");

    PathClient pc("mbf_costmap_nav/exe_path", true); // true doesnt need ros::spin

/*
     while(!pc.waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for Move Base server to come up");
     }

*/    
     if(!pc.waitForServer(ros::Duration(10.0))){
         ROS_INFO("Waiting for Move Base server to come up - waiting 20, then proceeding");
     } 

    mbf_msgs::ExePathGoal target_path_;

    nav_msgs::Path path_;

    int no_of_poses_in_path = 4;       // Number of poses to compose the path

    std::vector<geometry_msgs::PoseStamped> poses_(no_of_poses_in_path);

    // Fill the pose vector with zeroes 
     for (int i=0; i<no_of_poses_in_path; i++){
        memset(&poses_[i].pose.position, 0, 3);
        memset(&poses_[i].pose.orientation, 0, 4);

    }

    // Insert your series of poses here using your logic 
    // Here I am using 4 poses 50 cm apart
     for (int i=0; i<no_of_poses_in_path; i++){
        poses_[i].header.frame_id = "odom";
        poses_[i].pose.position.x = (0.5*i);
        poses_[i].pose.position.y = 0;
        poses_[i].pose.orientation.z = 0; 
        poses_[i].pose.orientation.w = 0;    
        poses_[i].header.stamp = ros::Time::now();

     }

   // Populate the Path Message
    path_.header.frame_id = "odom";
    path_.poses = poses_;
    path_.header.stamp = ros::Time::now();

    ROS_INFO("Goal Path Planned %f %f %f %f", path_.poses[0].pose.position.x, path_.poses[1].pose.position.x, 
                                         path_.poses[2].pose.position.x, path_.poses[3].pose.position.x);

   // Populate the controller and path fields (Refer to ExePath.Action)

    target_path_.controller = "TebLocalPlannerROS"; // As defined in list of controllers in your yaml

    target_path_.path = path_;

  // Interact with Action Server

  pc.sendGoal(target_path_);

  ROS_INFO("Goal sent");

  pc.waitForResult();
  if(pc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Base moved %s", pc.getState().toString().c_str());

  else if(pc.getState() == actionlib::SimpleClientGoalState::ABORTED)  
    ROS_INFO("Goal aborted");

  else 
    ROS_INFO("Base failed to move for some reason");


  ros::shutdown();
}