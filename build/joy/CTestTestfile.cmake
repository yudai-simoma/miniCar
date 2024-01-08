# CMake generated Testfile for 
# Source directory: /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/joy
# Build directory: /home/ubuntu/Desktop/catkin_ws/build/joy
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_joy_roslint_package "/home/ubuntu/Desktop/catkin_ws/build/joy/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/Desktop/catkin_ws/build/joy/test_results/joy/roslint-joy.xml" "--working-dir" "/home/ubuntu/Desktop/catkin_ws/build/joy" "--return-code" "/opt/ros/noetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/ubuntu/Desktop/catkin_ws/build/joy/test_results/joy/roslint-joy.xml make roslint_joy")
set_tests_properties(_ctest_joy_roslint_package PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslint/cmake/roslint-extras.cmake;67;catkin_run_tests_target;/home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/joy/CMakeLists.txt;17;roslint_add_test;/home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/joy/CMakeLists.txt;0;")
subdirs("gtest")
