# CMake generated Testfile for 
# Source directory: /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver
# Build directory: /home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_nmea_navsat_driver_venv_check_nmea_navsat_driver-requirements "/home/tractor/ros1_lawn_tractor_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver/venv_check-nmea_navsat_driver-requirements.xml" "--working-dir" "/home/tractor/ros1_lawn_tractor_ws/build" "--return-code" "/home/tractor/ros1_lawn_tractor_ws/build/catkin_generated/env_cached.sh rosrun catkin_virtualenv venv_check venv --requirements /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/requirements.txt         --extra-pip-args \"\\\"-qq --retries 10 --timeout 30\\\"\"         --xunit-output /home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver/venv_check-nmea_navsat_driver-requirements.xml")
set_tests_properties(_ctest_nmea_navsat_driver_venv_check_nmea_navsat_driver-requirements PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin_virtualenv/cmake/catkin_generate_virtualenv.cmake;157;catkin_run_tests_target;/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/CMakeLists.txt;37;catkin_generate_virtualenv;/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/CMakeLists.txt;0;")
add_test(_ctest_nmea_navsat_driver_nosetests_test.test_driver.py "/home/tractor/ros1_lawn_tractor_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver/nosetests-test.test_driver.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver" "/home/tractor/ros1_lawn_tractor_ws/devel/share/nmea_navsat_driver/venv/bin/python -m nose -P --process-timeout=60 /home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/test/test_driver.py --with-xunit --xunit-file=/home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver/nosetests-test.test_driver.py.xml")
set_tests_properties(_ctest_nmea_navsat_driver_nosetests_test.test_driver.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/CMakeLists.txt;41;catkin_add_nosetests;/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/CMakeLists.txt;0;")
add_test(_ctest_nmea_navsat_driver_roslint_package "/home/tractor/ros1_lawn_tractor_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver/roslint-nmea_navsat_driver.xml" "--working-dir" "/home/tractor/ros1_lawn_tractor_ws/build/nmea_navsat_driver" "--return-code" "/opt/ros/noetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/tractor/ros1_lawn_tractor_ws/build/test_results/nmea_navsat_driver/roslint-nmea_navsat_driver.xml make roslint_nmea_navsat_driver")
set_tests_properties(_ctest_nmea_navsat_driver_roslint_package PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslint/cmake/roslint-extras.cmake;67;catkin_run_tests_target;/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/CMakeLists.txt;53;roslint_add_test;/home/tractor/ros1_lawn_tractor_ws/src/nmea_navsat_driver/CMakeLists.txt;0;")
