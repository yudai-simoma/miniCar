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
CMAKE_SOURCE_DIR = /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/Desktop/catkin_ws/build/spacenav_node

# Include any dependencies generated for this target.
include CMakeFiles/spacenav_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/spacenav_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/spacenav_node.dir/flags.make

CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.o: CMakeFiles/spacenav_node.dir/flags.make
CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.o: /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node/src/spacenav_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/Desktop/catkin_ws/build/spacenav_node/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.o -c /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node/src/spacenav_node.cpp

CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node/src/spacenav_node.cpp > CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.i

CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node/src/spacenav_node.cpp -o CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.s

# Object files for target spacenav_node
spacenav_node_OBJECTS = \
"CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.o"

# External object files for target spacenav_node
spacenav_node_EXTERNAL_OBJECTS =

/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: CMakeFiles/spacenav_node.dir/src/spacenav_node.cpp.o
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: CMakeFiles/spacenav_node.dir/build.make
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/libroscpp.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/librosconsole.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/librostime.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /opt/ros/noetic/lib/libcpp_common.so
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node: CMakeFiles/spacenav_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/Desktop/catkin_ws/build/spacenav_node/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/spacenav_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/spacenav_node.dir/build: /home/ubuntu/Desktop/catkin_ws/devel/.private/spacenav_node/lib/spacenav_node/spacenav_node

.PHONY : CMakeFiles/spacenav_node.dir/build

CMakeFiles/spacenav_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/spacenav_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/spacenav_node.dir/clean

CMakeFiles/spacenav_node.dir/depend:
	cd /home/ubuntu/Desktop/catkin_ws/build/spacenav_node && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node /home/ubuntu/Desktop/catkin_ws/src/joystick_drivers/spacenav_node /home/ubuntu/Desktop/catkin_ws/build/spacenav_node /home/ubuntu/Desktop/catkin_ws/build/spacenav_node /home/ubuntu/Desktop/catkin_ws/build/spacenav_node/CMakeFiles/spacenav_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/spacenav_node.dir/depend

