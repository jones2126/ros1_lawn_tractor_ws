cmake_minimum_required(VERSION 3.0.2)
project(ackermann_vehicle)

#ref: http://wiki.ros.org/actionlib#Catkin

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base_msgs
  mbf_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
)

#generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
#   LIBRARIES ackermann_vehicle
   CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    move_base_msgs
    mbf_msgs
    geometry_msgs    
)

include_directories(${catkin_INCLUDE_DIRS})

#add_executable(ackermann_vehicle src/mbf_mission_simple.cpp)
add_executable(mbf_mission_simple src/mbf_mission_simple.cpp)
target_link_libraries(mbf_mission_simple ${catkin_LIBRARIES})
