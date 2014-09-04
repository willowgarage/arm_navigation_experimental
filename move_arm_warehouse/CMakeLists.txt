cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set( QT_WIDGET_SOURCES
  src/planning_scene_warehouse_viewer.cpp
)

set( QT_WIDGET_HEADERS
  include/move_arm_warehouse/planning_scene_warehouse_viewer.h
)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
#Fix to address https://bugreports.qt-project.org/browse/QTBUG-22829
qt4_wrap_cpp(moc_sources ${QT_WIDGET_HEADERS}
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

#rosbuild_genmsg()

#rosbuild_add_executable(move_arm_monitor src/move_arm_setup.cpp src/move_arm_monitor.cpp)
#rosbuild_link_boost(move_arm_monitor thread)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OCTOMAP REQUIRED octomap)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
    message(STATUS "Found assimp v3")
  else()
    message(STATUS "Found assimp v2")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

rosbuild_add_library(move_arm_logging src/move_arm_warehouse_logger_reader.cpp)
rosbuild_add_library(move_arm_utils src/move_arm_utils.cpp include/move_arm_warehouse/move_arm_utils.h)

rosbuild_add_executable(move_arm_simple_action src/move_arm_simple_action.cpp)

rosbuild_link_boost(move_arm_simple_action thread)
target_link_libraries(move_arm_utils move_arm_logging)
target_link_libraries(move_arm_simple_action move_arm_logging ${ASSIMP_LIBRARIES})

#rosbuild_add_executable(warehouse_components_visualizer src/warehouse_components_visualizer.cpp)
#target_link_libraries(warehouse_components_visualizer move_arm_logging)

rosbuild_add_executable(planning_scene_warehouse_viewer src/trajectory_stats.cpp ${QT_WIDGET_SOURCES} ${QT_WIDGET_HEADERS} ${moc_sources})
target_link_libraries(planning_scene_warehouse_viewer ${QT_LIBRARIES} move_arm_utils)

#rosbuild_add_rostest_labeled(test_utils launch/test_warehouse_viewer.launch)
rosbuild_add_executable(test_utils test/move_arm_utils_test.cpp)
target_link_libraries(test_utils move_arm_utils)
target_link_libraries(test_utils gtest)
