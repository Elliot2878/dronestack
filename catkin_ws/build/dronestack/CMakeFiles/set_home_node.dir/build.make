# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/cpsl/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/cpsl/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cpsl/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cpsl/catkin_ws/build

# Include any dependencies generated for this target.
include dronestack/CMakeFiles/set_home_node.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include dronestack/CMakeFiles/set_home_node.dir/compiler_depend.make

# Include the progress variables for this target.
include dronestack/CMakeFiles/set_home_node.dir/progress.make

# Include the compile flags for this target's objects.
include dronestack/CMakeFiles/set_home_node.dir/flags.make

dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.o: dronestack/CMakeFiles/set_home_node.dir/flags.make
dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.o: /home/cpsl/catkin_ws/src/dronestack/src/set_home.cpp
dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.o: dronestack/CMakeFiles/set_home_node.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/cpsl/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.o"
	cd /home/cpsl/catkin_ws/build/dronestack && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.o -MF CMakeFiles/set_home_node.dir/src/set_home.cpp.o.d -o CMakeFiles/set_home_node.dir/src/set_home.cpp.o -c /home/cpsl/catkin_ws/src/dronestack/src/set_home.cpp

dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/set_home_node.dir/src/set_home.cpp.i"
	cd /home/cpsl/catkin_ws/build/dronestack && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cpsl/catkin_ws/src/dronestack/src/set_home.cpp > CMakeFiles/set_home_node.dir/src/set_home.cpp.i

dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/set_home_node.dir/src/set_home.cpp.s"
	cd /home/cpsl/catkin_ws/build/dronestack && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cpsl/catkin_ws/src/dronestack/src/set_home.cpp -o CMakeFiles/set_home_node.dir/src/set_home.cpp.s

# Object files for target set_home_node
set_home_node_OBJECTS = \
"CMakeFiles/set_home_node.dir/src/set_home.cpp.o"

# External object files for target set_home_node
set_home_node_EXTERNAL_OBJECTS =

/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: dronestack/CMakeFiles/set_home_node.dir/src/set_home.cpp.o
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: dronestack/CMakeFiles/set_home_node.dir/build.make
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/liborocos-kdl.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/liborocos-kdl.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libtf2_ros.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libactionlib.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libmessage_filters.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libroscpp.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/librosconsole.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libtf2.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/librostime.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /opt/ros/noetic/lib/libcpp_common.so
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node: dronestack/CMakeFiles/set_home_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/cpsl/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node"
	cd /home/cpsl/catkin_ws/build/dronestack && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/set_home_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dronestack/CMakeFiles/set_home_node.dir/build: /home/cpsl/catkin_ws/devel/lib/dronestack/set_home_node
.PHONY : dronestack/CMakeFiles/set_home_node.dir/build

dronestack/CMakeFiles/set_home_node.dir/clean:
	cd /home/cpsl/catkin_ws/build/dronestack && $(CMAKE_COMMAND) -P CMakeFiles/set_home_node.dir/cmake_clean.cmake
.PHONY : dronestack/CMakeFiles/set_home_node.dir/clean

dronestack/CMakeFiles/set_home_node.dir/depend:
	cd /home/cpsl/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cpsl/catkin_ws/src /home/cpsl/catkin_ws/src/dronestack /home/cpsl/catkin_ws/build /home/cpsl/catkin_ws/build/dronestack /home/cpsl/catkin_ws/build/dronestack/CMakeFiles/set_home_node.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : dronestack/CMakeFiles/set_home_node.dir/depend
