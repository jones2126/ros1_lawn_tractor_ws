# Install script for directory: /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/tractor/ros1_lawn_tractor_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver/catkin_generated/installspace/nmea_navsat_driver.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nmea_navsat_driver/cmake" TYPE FILE FILES
    "/home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver/catkin_generated/installspace/nmea_navsat_driverConfig.cmake"
    "/home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver/catkin_generated/installspace/nmea_navsat_driverConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nmea_navsat_driver" TYPE FILE FILES "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nmea_navsat_driver" TYPE PROGRAM FILES
    "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/scripts/nmea_serial_driver"
    "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/scripts/nmea_socket_driver"
    "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/scripts/nmea_topic_driver"
    "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/scripts/nmea_topic_serial_reader"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nmea_navsat_driver/launch" TYPE DIRECTORY FILES "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/launch/" FILES_MATCHING REGEX "/[^/]*\\.launch$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nmea_navsat_driver" TYPE FILE FILES
    "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/LICENSE.txt"
    "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/requirements.txt"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nmea_navsat_driver" TYPE DIRECTORY FILES "/home/tractor/ros1_lawn_tractor_ws/build/install/venv" USE_SOURCE_PERMISSIONS)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/nmea_navsat_driver/catkin_virtualenv_scripts" TYPE FILE FILES "/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/test_driver.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/nmea_navsat_driver" TYPE PROGRAM FILES "/home/tractor/ros1_lawn_tractor_ws/build/test_driver.py")
endif()

