/* A ROS implementation of the Pure pursuit path tracking algorithm (Coulter 1992).

   Terminology (mostly :) follows:
   Coulter, Implementation of the pure pursuit algoritm, 1992 and 
   Sorniotti et al. Path tracking for Automated Driving, 2017.
 */

/* Copied from Juan's workspace

potential ref: https://github.com/6RiverSystems/pure_pursuit/blob/6e54d43e307edcde4a2fac41c0ad5cbb6e47dbae/src/pure_pursuit.cpp
potential ref: https://github.com/pasrom/pure_pursuit/blob/866b70b3b5fea1f94a3a1279f97fccb47616943f/src/pure_pursuit.cpp

change log:
12/5/23
- changed the name 'controller' to 'pure_pursuit_steering_controller' ; line 600, 602
- comment out ' void run();' - line 77 because that will be done by 'int main()'
- moved the constructor statements to 'public'
- Created an 'initialize' function and moved parameter setting and other one time statements here
- Add timer nh.createTimer(ros::Duration(10.0), std::bind(&PurePursuit::checkForNewPath, &controller)); inside the initialize function
- Change int main() to 'initialize; and addedd 'ros::spin();' starts the ROS event loop instead of function 'void PurePursuit::run()'
- Added 'ros::Timer timer_;' inside private class
- Added callback 'void checkForNewPathTimerCallback(const ros::TimerEvent&) {' inside the private class
- Added the function 'void PurePursuit::checkForNewPath() {'
- Change void PurePursuit::loadPathFromFile() { to accept the input file name as an arguement

 */

#include <string>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl/frames.hpp>

// TODO: Figure out how to use tf2 DataConversions
// for more elegant and compact code
//#include <tf2_kdl/tf2_kdl.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <pure_pursuit/PurePursuitConfig.h>
#include <fstream>
#include <tf/tf.h>
#include <iostream>
#include <std_msgs/String.h>

using std::string;

class PurePursuit
{
public:

  PurePursuit() : ld_(1.0),v_max_(0.1),v_(v_max_),w_max_(1.0),pos_tol_(0.1),idx_(0),goal_reached_(true),nh_private_("~"),
    tf_listener_(tf_buffer_), map_frame_id_("map"),robot_frame_id_("base_link"),lookahead_frame_id_("lookahead") 
    {ROS_INFO("In PurePursuit::PurePursuit constructor");} // <- end of constructor

  void initialize() {
    ROS_INFO("In initialize method");

    // Parameter-related statements
    nh_private_.param<double>("wheelbase", L_, 1.0);
    nh_private_.param<double>("lookahead_distance", ld_, 1.0);
    //nh_private_.param<double>("linear_velocity", v_, 0.1);
    nh_private_.param<double>("max_rotational_velocity", w_max_, 1.0);
    nh_private_.param<double>("position_tolerance", pos_tol_, 0.1);
    nh_private_.param<double>("steering_angle_velocity", delta_vel_, 100.0);
    nh_private_.param<double>("acceleration", acc_, 100.0);
    nh_private_.param<double>("jerk", jerk_, 100.0);
    nh_private_.param<double>("steering_angle_limit", delta_max_, 1.57);
    nh_private_.param<string>("map_frame_id", map_frame_id_, "map");
    // Frame attached to midpoint of rear axle (for front-steered vehicles).
    nh_private_.param<string>("robot_frame_id", robot_frame_id_, "base_link");
    // Lookahead frame moving along the path as the vehicle is moving.
    nh_private_.param<string>("lookahead_frame_id", lookahead_frame_id_, "lookahead");
    // Frame attached to midpoint of front axle (for front-steered vehicles).
    nh_private_.param<string>("ackermann_frame_id", acker_frame_id_, "rear_axle_midpoint");
    pub_multi_array_ = nh_.advertise<std_msgs::Float64MultiArray>("lookahead_data", 1);

    std::string default_input_file_path = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt";
    nh_private_.param<std::string>("mission_file_name", input_file_path_, default_input_file_path);


    // Populate messages with static data
    lookahead_.header.frame_id = robot_frame_id_;
    lookahead_.child_frame_id = lookahead_frame_id_;

    cmd_acker_.header.frame_id = acker_frame_id_;
    cmd_acker_.drive.steering_angle_velocity = delta_vel_;
    cmd_acker_.drive.acceleration = acc_;
    cmd_acker_.drive.jerk = jerk_;

    //sub_path_ = nh_.subscribe("path_segment", 1, &PurePursuit::receivePath, this);
    sub_odom_ = nh_.subscribe("odometry", 1, &PurePursuit::computeVelocities, this);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    pub_acker_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("cmd_acker", 1);
    //pub_got_path_ = nh_.advertise<std_msgs::Float64>("got_path", 1);
    pub_off_path_error_ = nh_.advertise<std_msgs::Float64>("off_path_error", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/drive_path", 1);  

    debug_info_pub = nh_.advertise<std_msgs::String>("pp_debug_info", 1);


    reconfigure_callback_ = boost::bind(&PurePursuit::reconfigure, this, _1, _2);
    reconfigure_server_.setCallback(reconfigure_callback_);    

    // Timer initialization to check for a new path
    timer_ = nh_.createTimer(ros::Duration(10.0), &PurePursuit::checkForNewPathTimerCallback, this);

    // Any other initialization tasks
    ROS_INFO("Loading the initial path file");
    debug_msg.data = "Loading the path file";
    debug_info_pub.publish(debug_msg);
    loadPathFromFile();
  }


  void computeVelocities(nav_msgs::Odometry odom);    // triggers each time new odometry data is received.
  void loadPathFromFile();                            // Loads path to follow from .txt file
  void checkForNewPath();                             // based on a timer, checks if there is a new path to follow

  //! Compute transform that transforms a pose into the robot frame (base_link)
  KDL::Frame transformToBaseLink(const geometry_msgs::Pose& pose, const geometry_msgs::Transform& tf);
  
  //! Helper founction for computing eucledian distances in the x-y plane.
  template<typename T1, typename T2>
  double distance(T1 pt1, T2 pt2)
  {
    return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
  }

  
private:

  ros::Timer timer_; // Timer declared as a private member variable

  // callback triggered by the timer
  void checkForNewPathTimerCallback(const ros::TimerEvent&) {
    ROS_INFO("callback triggered by timer - checking for new path");
    checkForNewPath();
  }


  ros::Publisher path_pub_; 

  //! Dynamic reconfigure callback.
  void reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level);

  void publishLookaheadData();
  
  // Vehicle parameters
  double L_;
  // Algorithm variables
  // Position tolerace is measured along the x-axis of the robot!
  double ld_, pos_tol_;
  double ld_2; //moving the definition here to see if I can publish it in the info array
  // Generic control variables
  double v_max_, v_, w_max_, ld_cfg_;
  // Control variables for Ackermann steering
  // Steering angle is denoted by delta
  double delta_, delta_vel_, acc_, jerk_, delta_max_;
  nav_msgs::Path path_;
  unsigned idx_;
  bool goal_reached_;
  int num_records;

  std::vector<double> lookahead_values_;
  std::vector<double> speed_values_;

  double yt = 0.0;
  geometry_msgs::Twist cmd_vel_;
  ackermann_msgs::AckermannDriveStamped cmd_acker_;
  //std_msgs::Float64 got_path_;
  std_msgs::Float64 off_path_error_;
  
  geometry_msgs::TransformStamped tf; // defined here so I can print debug messages
  // later this can be removed and defined in computeVelocities

  // Ros infrastructure
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber sub_odom_, sub_path_;
  ros::Publisher pub_vel_, pub_acker_, pub_off_path_error_;
  ros::Publisher pub_multi_array_;
  //ros::Publisher pub_got_path_;

  // for debugging purposes
  ros::Publisher debug_info_pub;
  std_msgs::String debug_msg;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped lookahead_;
  nav_msgs::Path loaded_path;  
  string map_frame_id_, robot_frame_id_, lookahead_frame_id_, acker_frame_id_;

  std::string input_file_path_;

  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig> reconfigure_server_;
  dynamic_reconfigure::Server<pure_pursuit::PurePursuitConfig>::CallbackType reconfigure_callback_;
  
};  // <-- Semicolon here to end the 'class PurePursuit' definition

void PurePursuit::computeVelocities(nav_msgs::Odometry odom)
{
  // The velocity commands are computed, each time a new Odometry message is received.
  // Odometry is not used directly, but through the tf tree.

  // Get the current robot pose
  // geometry_msgs::TransformStamped tf; // commented out here so I can print debug
  // messages which requires the definition to be class member variable tf throughout the class
  // later this can be re-enabled
  try
  {
    tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    // We first compute the new point to track, based on our current pose,
    // path information and lookahead distance.
    for (; idx_ < path_.poses.size(); idx_++)
    {
      v_ = speed_values_[idx_];
      ld_ = lookahead_values_[idx_];
      //ROS_INFO("inside idx loop: idx_ = %d, v_ = %f, ld_ = %f", idx_, v_, ld_);

      if (distance(path_.poses[idx_].pose.position, tf.transform.translation) > ld_)
      {

        // Transformed lookahead to base_link frame is lateral error
        // The pose data is the goal from the path data
        // The tf.transform is the current position
        KDL::Frame F_bl_ld = transformToBaseLink(path_.poses[idx_].pose, tf.transform);
        lookahead_.transform.translation.x = F_bl_ld.p.x();
        lookahead_.transform.translation.y = F_bl_ld.p.y();
        lookahead_.transform.translation.z = F_bl_ld.p.z();
        /*
        the GetQuaternion method extracts the rotation part of the frame and represents 
        it as a quaternion. 
        */
        F_bl_ld.M.GetQuaternion(lookahead_.transform.rotation.x,
                                lookahead_.transform.rotation.y,
                                lookahead_.transform.rotation.z,
                                lookahead_.transform.rotation.w);
        
        // TODO: See how the above conversion can be done more elegantly
        // using tf2_kdl and tf2_geometry_msgs

        break;
      }
    }

    if (!path_.poses.empty() && idx_ >= path_.poses.size())
    {
      // We are approaching the goal,
      // which is closer than ld

      // This is the pose of the goal w.r.t. the base_link frame
      KDL::Frame F_bl_end = transformToBaseLink(path_.poses.back().pose, tf.transform);

      if (fabs(F_bl_end.p.x()) <= pos_tol_)
      {
        // We have reached the goal
        goal_reached_ = true;

        // Reset the path
        path_ = nav_msgs::Path();
      }
      else
      {
        // We need to extend the lookahead distance
        // beyond the goal point.
      
        // Find the intersection between the circle of radius ld
        // centered at the robot (origin)
        // and the line defined by the last path pose
        double roll, pitch, yaw;
        F_bl_end.M.GetRPY(roll, pitch, yaw);
        double k_end = tan(yaw); // Slope of line defined by the last path pose
        double l_end = F_bl_end.p.y() - k_end * F_bl_end.p.x();
        double a = 1 + k_end * k_end;
        double b = 2 * l_end;
        double c = l_end * l_end - ld_ * ld_;
        double D = sqrt(b*b - 4*a*c);
        double x_ld = (-b + copysign(D,v_)) / (2*a);
        double y_ld = k_end * x_ld + l_end;
        
        lookahead_.transform.translation.x = x_ld;
        lookahead_.transform.translation.y = y_ld;
        lookahead_.transform.translation.z = F_bl_end.p.z();
        F_bl_end.M.GetQuaternion(lookahead_.transform.rotation.x,
                                 lookahead_.transform.rotation.y,
                                 lookahead_.transform.rotation.z,
                                 lookahead_.transform.rotation.w);
      }
    }

    if (!goal_reached_)
    {
      // We are tracking.

      // Compute linear velocity.
      // Right now,this is not very smart :)
      //v_ = copysign(v_max_, v_);
      
      // Compute the angular velocity.
      // Lateral error is the y-value of the lookahead point (in base_link frame)
      yt = lookahead_.transform.translation.y;
      //double ld_2 = ld_ * ld_;
      ld_2 = ld_ * ld_;
      //ROS_WARN("Lateral error  %.2f", yt);
      //ROS_INFO("Lateral error (y-value of lookahead point.) to %.2f", yt);
      off_path_error_.data = yt;
      pub_off_path_error_.publish(off_path_error_);
      /*
      // original version
      cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );
      
      second version is below, but I decided I don't want the low level controller
      to use a rotational angle based on speed, but rather a steer angle
      which matches the Ackermann steering angle so I'm going to comment
      out 
      if (yt >= 0){
        cmd_vel_.angular.z = std::min( 2*v_ / ld_2 * yt, w_max_ );
      } else {
        cmd_vel_.angular.z = std::max( 2*v_ / ld_2 * yt, -w_max_ );
      }
      */

      //ROS_WARN("angular z to %.2f", cmd_vel_.angular.z);

      // Compute desired Ackermann steering angle
      //cmd_acker_.drive.steering_angle = std::min( atan2(2 * yt * L_, ld_2), delta_max_ );  // original
      cmd_acker_.drive.steering_angle = atan2(2 * yt * L_, ld_2);
      if (cmd_acker_.drive.steering_angle > delta_max_){
          cmd_acker_.drive.steering_angle = delta_max_;
      } else if (cmd_acker_.drive.steering_angle < -delta_max_){
          cmd_acker_.drive.steering_angle = -delta_max_;
      }
      cmd_vel_.angular.z = cmd_acker_.drive.steering_angle;   // for expediency I'm using angular.z to hold steer angle

      // Set linear velocity for tracking.
      cmd_vel_.linear.x = v_;
      cmd_acker_.drive.speed = v_;

      cmd_acker_.header.stamp = ros::Time::now();
    }
    else
    {
      // We are at the goal!

      // Stop the vehicle
      
      // The lookahead target is at our current pose.
      lookahead_.transform = geometry_msgs::Transform();
      lookahead_.transform.rotation.w = 1.0;
      
      // Stop moving.
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;

      cmd_acker_.header.stamp = ros::Time::now();
      cmd_acker_.drive.steering_angle = 0.0;
      cmd_acker_.drive.speed = 0.0;
    }

    // Publish the lookahead target transform.
    lookahead_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(lookahead_);
    
    // Publish the velocities
    //cmd_vel_.linear.y = 9.9;      // only for testing
    pub_vel_.publish(cmd_vel_);
    
    // Publish ackerman steering setpoints
    pub_acker_.publish(cmd_acker_);

    // Publish the lookahead data
    publishLookaheadData();

  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }
}

void PurePursuit::loadPathFromFile() {

    debug_msg.data = "In the function loadPathFromFile";
    debug_info_pub.publish(debug_msg);

    // got_path_.data = 1;
    // pub_got_path_.publish(got_path_);
    ROS_INFO("In loadPathFromFile - about to open path file: %s", input_file_path_.c_str());
    std::ifstream file(input_file_path_);


    // std::string input_file = "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/input_path.txt";   
    // std::ifstream file(input_file);

    if (!file.is_open()) {
        ROS_ERROR("Failed to open path file: %s", input_file_path_.c_str());
        return;
    }

    double x, y, yaw, lookahead, speed;
    loaded_path.poses.clear();
    loaded_path.header.frame_id = "map";
    loaded_path.header.stamp = ros::Time::now();
    int seq = 0;

    while (file >> x >> y >> yaw >> lookahead >> speed) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.seq = seq;
        pose.header.stamp = loaded_path.header.stamp;        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        loaded_path.poses.push_back(pose);
        seq++;

        lookahead_values_.push_back(lookahead);
        speed_values_.push_back(speed);        
    }

    file.close();
    path_ = loaded_path;  // Assign the loaded path to your class member

    // Publish the path
    ROS_INFO("publishing path, 1st attempt");
    path_pub_.publish(loaded_path);
    ros::Duration(1.0).sleep();  // Sleep for 1 second
    ROS_INFO("publishing path, 2nd attempt");
    path_pub_.publish(loaded_path);
    ROS_INFO("publishing path done");    

    num_records = path_.poses.size();
    idx_ = 0;
    if (num_records > 0)
    {
      ROS_INFO("Number of records in path_: %d", num_records);
      goal_reached_ = false;
    }
    else
    {
      goal_reached_ = true;
      ROS_WARN_STREAM("Received empty path!");
    }    
}

// checkForNewPath method is looking for a parameter named "mission_file_name", you can set
// this parameter from the command line as follows:
// $ rosparam set /mission_file_name "/home/tractor/ros1_lawn_tractor_ws/project_notes/paths/Collins_Dr_62/ready_to_test/test_PID_output.txt"

void PurePursuit::checkForNewPath() {
  std::string new_input_file;
  if (ros::param::get("mission_file_name", input_file_path_) && !input_file_path_.empty()) {
      // Load the new path
      loadPathFromFile();

      ROS_INFO("New path file received: %s", new_input_file.c_str());
      debug_msg.data = "In the function checkForNewPath - New path file received: ";
      debug_info_pub.publish(debug_msg);

      // Reset the parameter
      ros::param::set("mission_file_name", "");
  }
}

KDL::Frame PurePursuit::transformToBaseLink(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Transform& tf)
{
  // Pose in global (map) frame
  KDL::Frame F_map_pose(KDL::Rotation::Quaternion(pose.orientation.x,
                                                  pose.orientation.y,
                                                  pose.orientation.z,
                                                  pose.orientation.w),
                        KDL::Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z));

  // Robot (base_link) in global (map) frame
  KDL::Frame F_map_tf(KDL::Rotation::Quaternion(tf.rotation.x,
                                                tf.rotation.y,
                                                tf.rotation.z,
                                                tf.rotation.w),
                      KDL::Vector(tf.translation.x,
                                  tf.translation.y,
                                  tf.translation.z));

  // TODO: See how the above conversions can be done more elegantly
  // using tf2_kdl and tf2_geometry_msgs

  return F_map_tf.Inverse()*F_map_pose;  // calculates the transformation from the the robot to the pose.
}

void PurePursuit::reconfigure(pure_pursuit::PurePursuitConfig &config, uint32_t level)
{
  v_max_ = config.max_linear_velocity;
  //ld_cfg_ = config.lookahead_distance;
}

void PurePursuit::publishLookaheadData()
{
	std_msgs::Float64MultiArray multi_array;
	// Convert lookahead point to map frame in order to print x, y position
	KDL::Frame F_map_robot(KDL::Rotation::Quaternion(tf.transform.rotation.x,
	                                               tf.transform.rotation.y,
	                                               tf.transform.rotation.z,
	                                               tf.transform.rotation.w),
	                     KDL::Vector(tf.transform.translation.x,
	                                 tf.transform.translation.y,
	                                 tf.transform.translation.z));

	KDL::Vector F_bl_lookahead(lookahead_.transform.translation.x,
	                         lookahead_.transform.translation.y,
	                         lookahead_.transform.translation.z);

	KDL::Vector F_map_lookahead = F_map_robot * F_bl_lookahead;

	multi_array.data.push_back(F_map_lookahead.x()); // Lookahead x position in map frame
	multi_array.data.push_back(F_map_lookahead.y()); // Lookahead y position in map frame

	// previously I published the lookahead position in the robot's frame (base_link) 
	//multi_array.data.push_back(lookahead_.transform.translation.x);  // this may be the goal in the tractor frame
	//multi_array.data.push_back(lookahead_.transform.translation.y);
	//multi_array.data.push_back(tf.transform.translation.x); // current x position
	//multi_array.data.push_back(tf.transform.translation.y); // current y position
	multi_array.data.push_back(cmd_acker_.drive.steering_angle);
	multi_array.data.push_back(cmd_vel_.angular.z);
	multi_array.data.push_back(yt);
	multi_array.data.push_back(L_);
	multi_array.data.push_back(ld_);
  multi_array.data.push_back(ld_2);
//cmd_acker_.drive.steering_angle = atan2(2 * yt * L_, ld_2);
	if(!path_.poses.empty()) {
		double x = path_.poses[idx_].pose.position.x;
		double y = path_.poses[idx_].pose.position.y;
		multi_array.data.push_back(idx_);
		multi_array.data.push_back(x);  // goal in map frame in path_.poses vector 
		multi_array.data.push_back(y);
		multi_array.data.push_back(path_.poses[idx_].pose.position.z);
		multi_array.data.push_back(path_.poses[idx_].pose.orientation.x);
		multi_array.data.push_back(path_.poses[idx_].pose.orientation.y);
		multi_array.data.push_back(path_.poses[idx_].pose.orientation.z);
		multi_array.data.push_back(path_.poses[idx_].pose.orientation.w);
	} else {
		multi_array.data.push_back(0);  // Add 0 idx_ position
		multi_array.data.push_back(0);  // Add 0 for x position
		multi_array.data.push_back(0);  // Add 0 for y position
		multi_array.data.push_back(0);	// Add 0 for pose.position.z
		multi_array.data.push_back(0);	// Add 0 for pose.orientation.x
		multi_array.data.push_back(0);	// Add 0 for pose.orientation.y
		multi_array.data.push_back(0);	// Add 0 for pose.orientation.z
		multi_array.data.push_back(0);	// Add 0 for pose.orientation.w
	}

	multi_array.data.push_back(tf.transform.translation.x);  // current x position 
	multi_array.data.push_back(tf.transform.translation.y);
	multi_array.data.push_back(tf.transform.translation.z);
	multi_array.data.push_back(tf.transform.rotation.x);
	multi_array.data.push_back(tf.transform.rotation.y);
	multi_array.data.push_back(tf.transform.rotation.z);
	multi_array.data.push_back(tf.transform.rotation.w);

  multi_array.data.push_back(lookahead_.transform.translation.x);  // distance and heading to get to goal???
  multi_array.data.push_back(lookahead_.transform.translation.y);
  multi_array.data.push_back(lookahead_.transform.translation.z);
  multi_array.data.push_back(lookahead_.transform.rotation.x);
  multi_array.data.push_back(lookahead_.transform.rotation.y);
  multi_array.data.push_back(lookahead_.transform.rotation.z);
  multi_array.data.push_back(lookahead_.transform.rotation.w);

	pub_multi_array_.publish(multi_array);
}


int main(int argc, char**argv)
{

  std::cout << "pure_pursuit.cpp program initiating..." << std::endl;
  ros::init(argc, argv, "pure_pursuit");
  ROS_INFO("PurePursuit controller starting");
  PurePursuit pure_pursuit_steering_controller;
  pure_pursuit_steering_controller.initialize(); // Initialize parameters and timer after ros::init()
  ros::spin();
  return 0;
}
