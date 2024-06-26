cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

# Set the build type to Debug to include debug symbols
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  ackermann_msgs
  tf2
  tf2_ros
  rospy
  dynamic_reconfigure
)

find_package(orocos_kdl REQUIRED)

############################
## dynamic reconfiure api ##
############################
generate_dynamic_reconfigure_options(
  cfg/PurePursuit.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs ackermann_msgs tf2 tf2_ros rospy dynamic_reconfigure
  )


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# executable for pure_pursuit.cpp
add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

# executable for test_kdl_transformToBaseLink
add_executable(test_kdl_transformToBaseLink src/test_kdl_transformToBaseLink.cpp)
target_link_libraries(test_kdl_transformToBaseLink ${catkin_LIBRARIES} orocos-kdl)

## Make sure configure headers are built before any node using them
add_dependencies(pure_pursuit ${PROJECT_NAME}_gencfg)

# Add the executable for simple_ros_node
add_executable(simple_ros_node src/simple_ros_node.cpp)
target_link_libraries(simple_ros_node ${catkin_LIBRARIES})
  
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lattice_planner lattice_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lattice_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
