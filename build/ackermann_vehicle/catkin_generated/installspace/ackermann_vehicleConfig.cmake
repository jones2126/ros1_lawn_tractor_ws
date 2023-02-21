# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(ackermann_vehicle_CONFIG_INCLUDED)
  return()
endif()
set(ackermann_vehicle_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("FALSE" STREQUAL "TRUE")
  set(ackermann_vehicle_SOURCE_PREFIX /home/tractor/ros1_lawn_tractor_ws/src/ackermann_vehicle)
  set(ackermann_vehicle_DEVEL_PREFIX /home/tractor/ros1_lawn_tractor_ws/devel)
  set(ackermann_vehicle_INSTALL_PREFIX "")
  set(ackermann_vehicle_PREFIX ${ackermann_vehicle_DEVEL_PREFIX})
else()
  set(ackermann_vehicle_SOURCE_PREFIX "")
  set(ackermann_vehicle_DEVEL_PREFIX "")
  set(ackermann_vehicle_INSTALL_PREFIX /home/tractor/ros1_lawn_tractor_ws/install)
  set(ackermann_vehicle_PREFIX ${ackermann_vehicle_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'ackermann_vehicle' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(ackermann_vehicle_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(ackermann_vehicle_INCLUDE_DIRS "")
  set(_include_dirs "")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT "https://github.com/jones2126/tractor_laptop_ROS_workspace " STREQUAL " ")
    set(_report "Check the website 'https://github.com/jones2126/tractor_laptop_ROS_workspace' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'al <aej2126@pm.me>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${ackermann_vehicle_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'ackermann_vehicle' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'ackermann_vehicle' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
    endif()
    _list_append_unique(ackermann_vehicle_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND ackermann_vehicle_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND ackermann_vehicle_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT ackermann_vehicle_NUM_DUMMY_TARGETS)
      set(ackermann_vehicle_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::ackermann_vehicle::wrapped-linker-option${ackermann_vehicle_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR ackermann_vehicle_NUM_DUMMY_TARGETS "${ackermann_vehicle_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::ackermann_vehicle::wrapped-linker-option${ackermann_vehicle_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND ackermann_vehicle_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND ackermann_vehicle_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND ackermann_vehicle_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/tractor/ros1_lawn_tractor_ws/install/lib;/home/tractor/ros1_lawn_tractor_ws/devel/lib;/opt/ros/noetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(ackermann_vehicle_LIBRARY_DIRS ${lib_path})
      list(APPEND ackermann_vehicle_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'ackermann_vehicle'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND ackermann_vehicle_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(ackermann_vehicle_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${ackermann_vehicle_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;rospy;std_msgs;move_base_msgs;mbf_msgs;geometry_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 ackermann_vehicle_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${ackermann_vehicle_dep}_FOUND)
      find_package(${ackermann_vehicle_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${ackermann_vehicle_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(ackermann_vehicle_INCLUDE_DIRS ${${ackermann_vehicle_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(ackermann_vehicle_LIBRARIES ${ackermann_vehicle_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${ackermann_vehicle_dep}_LIBRARIES})
  _list_append_deduplicate(ackermann_vehicle_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(ackermann_vehicle_LIBRARIES ${ackermann_vehicle_LIBRARIES})

  _list_append_unique(ackermann_vehicle_LIBRARY_DIRS ${${ackermann_vehicle_dep}_LIBRARY_DIRS})
  _list_append_deduplicate(ackermann_vehicle_EXPORTED_TARGETS ${${ackermann_vehicle_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${ackermann_vehicle_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
