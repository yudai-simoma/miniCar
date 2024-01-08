# CMake generated Testfile for 
# Source directory: /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/wiimote
# Build directory: /home/ubuntu/Desktop/catkin_ws/build/wiimote
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_wiimote_roslint_package "/home/ubuntu/Desktop/catkin_ws/build/wiimote/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/Desktop/catkin_ws/build/wiimote/test_results/wiimote/roslint-wiimote.xml" "--working-dir" "/home/ubuntu/Desktop/catkin_ws/build/wiimote" "--return-code" "/opt/ros/noetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/ubuntu/Desktop/catkin_ws/build/wiimote/test_results/wiimote/roslint-wiimote.xml make roslint_wiimote")
set_tests_properties(_ctest_wiimote_roslint_package PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslint/cmake/roslint-extras.cmake;67;catkin_run_tests_target;/home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/wiimote/CMakeLists.txt;77;roslint_add_test;/home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/wiimote/CMakeLists.txt;0;")
subdirs("gtest")
