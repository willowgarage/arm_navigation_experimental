cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#rosbuild_genmsg()

#rosbuild_add_executable(move_arm_monitor src/move_arm_setup.cpp src/move_arm_monitor.cpp)
#rosbuild_link_boost(move_arm_monitor thread)

rosbuild_add_library(move_arm_logging src/move_arm_warehouse_logger_reader.cpp)


rosbuild_add_executable(move_arm_simple_action src/move_arm_simple_action.cpp)
rosbuild_add_executable(warehouse_components_visualizer src/warehouse_components_visualizer.cpp)
rosbuild_link_boost(move_arm_simple_action thread)
target_link_libraries(move_arm_simple_action move_arm_logging)
target_link_libraries(warehouse_components_visualizer move_arm_logging)


