# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tractor/ros1_lawn_tractor_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tractor/ros1_lawn_tractor_ws/build

# Utility rule file for nmea_navsat_driver_generate_virtualenv.

# Include the progress variables for this target.
include nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/progress.make

nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv: /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv
nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv: nmea_navsat_driver/install/venv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tractor/ros1_lawn_tractor_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Per-package virtualenv target"

/home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv: venv/bin/activate
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tractor/ros1_lawn_tractor_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Prepare relocated virtualenvs for develspace and installspace"
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && mkdir -p /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv && cp -r venv/* /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && mkdir -p install/venv && cp -r venv/* install/venv
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && ../catkin_generated/env_cached.sh rosrun catkin_virtualenv venv_relocate /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv --target-dir /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && ../catkin_generated/env_cached.sh rosrun catkin_virtualenv venv_relocate install/venv --target-dir /home/tractor/ros1_lawn_tractor_ws/install/share/nmea_navsat_driver/venv

nmea_navsat_driver/install/venv: /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv
	@$(CMAKE_COMMAND) -E touch_nocreate nmea_navsat_driver/install/venv

venv/bin/activate: venv/bin/python
venv/bin/activate: /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/requirements.txt
venv/bin/activate: /opt/ros/noetic/share/catkin_virtualenv/requirements.txt
venv/bin/activate: /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/requirements.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tractor/ros1_lawn_tractor_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Install requirements to /home/tractor/ros1_lawn_tractor_ws/build/venv"
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && ../catkin_generated/env_cached.sh rosrun catkin_virtualenv venv_install venv --requirements /opt/ros/noetic/share/catkin_virtualenv/requirements.txt /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/requirements.txt --extra-pip-args \"-qq\ --retries\ 10\ --timeout\ 30\"

venv/bin/python:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tractor/ros1_lawn_tractor_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generate virtualenv in /home/tractor/ros1_lawn_tractor_ws/build/venv"
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && ../catkin_generated/env_cached.sh rosrun catkin_virtualenv venv_init venv --python python3 --use-system-packages --extra-pip-args \"-qq\ --retries\ 10\ --timeout\ 30\"

nmea_navsat_driver_generate_virtualenv: nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv
nmea_navsat_driver_generate_virtualenv: /home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv
nmea_navsat_driver_generate_virtualenv: nmea_navsat_driver/install/venv
nmea_navsat_driver_generate_virtualenv: venv/bin/activate
nmea_navsat_driver_generate_virtualenv: venv/bin/python
nmea_navsat_driver_generate_virtualenv: nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/build.make

.PHONY : nmea_navsat_driver_generate_virtualenv

# Rule to build all files generated by this target.
nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/build: nmea_navsat_driver_generate_virtualenv

.PHONY : nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/build

nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/clean:
	cd /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver && $(CMAKE_COMMAND) -P CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/cmake_clean.cmake
.PHONY : nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/clean

nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/depend:
	cd /home/tractor/ros1_lawn_tractor_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tractor/ros1_lawn_tractor_ws/src /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver /home/tractor/ros1_lawn_tractor_ws/build /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : nmea_navsat_driver/CMakeFiles/nmea_navsat_driver_generate_virtualenv.dir/depend
