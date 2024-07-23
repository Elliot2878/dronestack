# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/orinnano/SDK_ros2_ws/src/mavlink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/orinnano/SDK_ros2_ws/build/mavlink

# Utility rule file for AVSSUAS.xml-v1.0.

# Include any custom commands dependencies for this target.
include CMakeFiles/AVSSUAS.xml-v1.0.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/AVSSUAS.xml-v1.0.dir/progress.make

CMakeFiles/AVSSUAS.xml-v1.0: include/v1.0/AVSSUAS/AVSSUAS.h

include/v1.0/AVSSUAS/AVSSUAS.h: /home/orinnano/SDK_ros2_ws/src/mavlink/message_definitions/v1.0/AVSSUAS.xml
include/v1.0/AVSSUAS/AVSSUAS.h: /home/orinnano/SDK_ros2_ws/src/mavlink/message_definitions/v1.0/common.xml
include/v1.0/AVSSUAS/AVSSUAS.h: /home/orinnano/SDK_ros2_ws/src/mavlink/pymavlink/tools/mavgen.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/orinnano/SDK_ros2_ws/build/mavlink/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating include/v1.0/AVSSUAS/AVSSUAS.h"
	/usr/bin/env PYTHONPATH="/home/orinnano/SDK_ros2_ws/src/mavlink:/home/orinnano/SDK_ros2_ws/install/mavros/local/lib/python3.10/dist-packages:/home/orinnano/SDK_ros2_ws/install/mavros_msgs/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/rqt_bag_plugins/src:/home/orinnano/ros2_humble/install/rqt_bag_plugins/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_bag/src:/home/orinnano/ros2_humble/install/rqt_bag/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2bag:/home/orinnano/ros2_humble/install/ros2bag/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/rosbag2_py/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/interactive_markers/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/visualization_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/turtlesim/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/tracetools_test:/home/orinnano/ros2_humble/install/tracetools_test/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/tracetools_launch:/home/orinnano/ros2_humble/install/tracetools_launch/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/topic_monitor:/home/orinnano/ros2_humble/install/topic_monitor/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/tf2_tools:/home/orinnano/ros2_humble/install/tf2_tools/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/tf2_kdl/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/tf2_geometry_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/examples_tf2_py:/home/orinnano/ros2_humble/install/examples_tf2_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/tf2_ros_py:/home/orinnano/ros2_humble/install/tf2_ros_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/tf2_py/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/tf2_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/test_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/sros2:/home/orinnano/ros2_humble/install/sros2/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_topic/src:/home/orinnano/ros2_humble/install/rqt_topic/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_srv/src:/home/orinnano/ros2_humble/install/rqt_srv/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_shell/src:/home/orinnano/ros2_humble/install/rqt_shell/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_service_caller/src:/home/orinnano/ros2_humble/install/rqt_service_caller/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_reconfigure/src:/home/orinnano/ros2_humble/install/rqt_reconfigure/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_py_console/src:/home/orinnano/ros2_humble/install/rqt_py_console/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_publisher/src:/home/orinnano/ros2_humble/install/rqt_publisher/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_plot/src:/home/orinnano/ros2_humble/install/rqt_plot/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_action/src:/home/orinnano/ros2_humble/install/rqt_action/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_msg/src:/home/orinnano/ros2_humble/install/rqt_msg/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_console/src:/home/orinnano/ros2_humble/install/rqt_console/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt:/home/orinnano/ros2_humble/install/rqt/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/rqt_py_common/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/rqt_graph/src:/home/orinnano/ros2_humble/install/rqt_graph/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_gui_py/src:/home/orinnano/ros2_humble/install/rqt_gui_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/rqt_gui/src:/home/orinnano/ros2_humble/install/rqt_gui/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/rosbag2_storage_mcap_testdata/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/ros2trace:/home/orinnano/ros2_humble/install/ros2trace/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2topic:/home/orinnano/ros2_humble/install/ros2topic/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2test:/home/orinnano/ros2_humble/install/ros2test/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2component:/home/orinnano/ros2_humble/install/ros2component/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2param:/home/orinnano/ros2_humble/install/ros2param/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2lifecycle:/home/orinnano/ros2_humble/install/ros2lifecycle/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2service:/home/orinnano/ros2_humble/install/ros2service/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2run:/home/orinnano/ros2_humble/install/ros2run/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2launch:/home/orinnano/ros2_humble/install/ros2launch/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2pkg:/home/orinnano/ros2_humble/install/ros2pkg/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2node:/home/orinnano/ros2_humble/install/ros2node/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2multicast:/home/orinnano/ros2_humble/install/ros2multicast/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2interface:/home/orinnano/ros2_humble/install/ros2interface/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2doctor:/home/orinnano/ros2_humble/install/ros2doctor/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/ros2cli_test_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/ros2action:/home/orinnano/ros2_humble/install/ros2action/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ros2cli:/home/orinnano/ros2_humble/install/ros2cli/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/quality_of_service_demo_py:/home/orinnano/ros2_humble/install/quality_of_service_demo_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/message_filters/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/lifecycle_py:/home/orinnano/ros2_humble/install/lifecycle_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch_testing_ros:/home/orinnano/ros2_humble/install/launch_testing_ros/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch_ros:/home/orinnano/ros2_humble/install/launch_ros/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/laser_geometry/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/examples_rclpy_pointcloud_publisher:/home/orinnano/ros2_humble/install/examples_rclpy_pointcloud_publisher/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_minimal_subscriber:/home/orinnano/ros2_humble/install/examples_rclpy_minimal_subscriber/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_minimal_service:/home/orinnano/ros2_humble/install/examples_rclpy_minimal_service/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_minimal_publisher:/home/orinnano/ros2_humble/install/examples_rclpy_minimal_publisher/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_minimal_client:/home/orinnano/ros2_humble/install/examples_rclpy_minimal_client/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_minimal_action_server:/home/orinnano/ros2_humble/install/examples_rclpy_minimal_action_server/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_minimal_action_client:/home/orinnano/ros2_humble/install/examples_rclpy_minimal_action_client/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_guard_conditions:/home/orinnano/ros2_humble/install/examples_rclpy_guard_conditions/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/examples_rclpy_executors:/home/orinnano/ros2_humble/install/examples_rclpy_executors/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/demo_nodes_py:/home/orinnano/ros2_humble/install/demo_nodes_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/action_tutorials_py:/home/orinnano/ros2_humble/install/action_tutorials_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/rclpy/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/example_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/action_tutorials_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/action_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/unique_identifier_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/ament_uncrustify:/home/orinnano/ros2_humble/install/ament_uncrustify/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/trajectory_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/tracetools_trace:/home/orinnano/ros2_humble/install/tracetools_trace/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/tracetools_read:/home/orinnano/ros2_humble/install/tracetools_read/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/logging_demo/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/qt_gui_cpp/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/test_tracetools_launch:/home/orinnano/ros2_humble/install/test_tracetools_launch/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/test_launch_ros:/home/orinnano/ros2_humble/install/test_launch_ros/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/qt_gui/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/stereo_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/std_srvs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/shape_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/sensor_msgs_py:/home/orinnano/ros2_humble/install/sensor_msgs_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/map_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/sensor_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/nav_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/diagnostic_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/geometry_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/actionlib_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/std_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/statistics_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosgraph_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosbag2_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rmw_dds_common/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/composition_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rcl_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/pendulum_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/lifecycle_msgs/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/builtin_interfaces/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_generator_py/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/rpyutils:/home/orinnano/ros2_humble/install/rpyutils/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/rosidl_typesupport_cpp/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_typesupport_introspection_cpp/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_typesupport_c/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_typesupport_introspection_c/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_typesupport_fastrtps_c/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_generator_cpp/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_generator_c/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/rosidl_runtime_py:/home/orinnano/ros2_humble/install/rosidl_runtime_py/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/rosidl_generator_dds_idl/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_cmake/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_parser/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rosidl_adapter/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/rosidl_cli:/home/orinnano/ros2_humble/install/rosidl_cli/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/resource_retriever/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/rcutils/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/qt_gui_py_common/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/qt_dotgraph/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/install/python_qt_binding/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/launch_pytest:/home/orinnano/ros2_humble/install/launch_pytest/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch_testing:/home/orinnano/ros2_humble/install/launch_testing/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch_yaml:/home/orinnano/ros2_humble/install/launch_yaml/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch_xml:/home/orinnano/ros2_humble/install/launch_xml/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch:/home/orinnano/ros2_humble/install/launch/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/osrf_pycommon:/home/orinnano/ros2_humble/install/osrf_pycommon/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/launch_testing_examples:/home/orinnano/ros2_humble/install/launch_testing_examples/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/ament_cmake_google_benchmark/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/domain_coordinator:/home/orinnano/ros2_humble/install/domain_coordinator/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_xmllint:/home/orinnano/ros2_humble/install/ament_xmllint/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_pyflakes:/home/orinnano/ros2_humble/install/ament_pyflakes/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_pycodestyle:/home/orinnano/ros2_humble/install/ament_pycodestyle/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_pep257:/home/orinnano/ros2_humble/install/ament_pep257/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_pclint:/home/orinnano/ros2_humble/install/ament_pclint/lib/python3.10/site-packages:/home/orinnano/ros2_humble/install/ament_cmake_test/local/lib/python3.10/dist-packages:/home/orinnano/ros2_humble/build/ament_package:/home/orinnano/ros2_humble/install/ament_package/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_mypy:/home/orinnano/ros2_humble/install/ament_mypy/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_lint_cmake:/home/orinnano/ros2_humble/install/ament_lint_cmake/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_flake8:/home/orinnano/ros2_humble/install/ament_flake8/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_copyright:/home/orinnano/ros2_humble/install/ament_copyright/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_lint:/home/orinnano/ros2_humble/install/ament_lint/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_index_python:/home/orinnano/ros2_humble/install/ament_index_python/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_cpplint:/home/orinnano/ros2_humble/install/ament_cpplint/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_cppcheck:/home/orinnano/ros2_humble/install/ament_cppcheck/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_clang_tidy:/home/orinnano/ros2_humble/install/ament_clang_tidy/lib/python3.10/site-packages:/home/orinnano/ros2_humble/build/ament_clang_format:/home/orinnano/ros2_humble/install/ament_clang_format/lib/python3.10/site-packages" /usr/bin/python3.10 /home/orinnano/SDK_ros2_ws/src/mavlink/pymavlink/tools/mavgen.py --lang=C --wire-protocol=1.0 --output=include/v1.0 /home/orinnano/SDK_ros2_ws/src/mavlink/message_definitions/v1.0/AVSSUAS.xml

AVSSUAS.xml-v1.0: CMakeFiles/AVSSUAS.xml-v1.0
AVSSUAS.xml-v1.0: include/v1.0/AVSSUAS/AVSSUAS.h
AVSSUAS.xml-v1.0: CMakeFiles/AVSSUAS.xml-v1.0.dir/build.make
.PHONY : AVSSUAS.xml-v1.0

# Rule to build all files generated by this target.
CMakeFiles/AVSSUAS.xml-v1.0.dir/build: AVSSUAS.xml-v1.0
.PHONY : CMakeFiles/AVSSUAS.xml-v1.0.dir/build

CMakeFiles/AVSSUAS.xml-v1.0.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/AVSSUAS.xml-v1.0.dir/cmake_clean.cmake
.PHONY : CMakeFiles/AVSSUAS.xml-v1.0.dir/clean

CMakeFiles/AVSSUAS.xml-v1.0.dir/depend:
	cd /home/orinnano/SDK_ros2_ws/build/mavlink && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/orinnano/SDK_ros2_ws/src/mavlink /home/orinnano/SDK_ros2_ws/src/mavlink /home/orinnano/SDK_ros2_ws/build/mavlink /home/orinnano/SDK_ros2_ws/build/mavlink /home/orinnano/SDK_ros2_ws/build/mavlink/CMakeFiles/AVSSUAS.xml-v1.0.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/AVSSUAS.xml-v1.0.dir/depend

