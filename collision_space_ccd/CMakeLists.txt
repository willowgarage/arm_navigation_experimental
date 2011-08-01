cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -march=native -mfpmath=sse")



rosbuild_add_boost_directories()

rosbuild_add_library(collision_space_ccd 
src/environment.cpp 
src/environment_objects.cpp 
src/environmentBVH.cpp 
src/interval_tree.cpp)

#set(THIRDPARTY_PATH /wg/stor1a/jpan/Workspace/thirdparty/)
#include_directories(${THIRDPARTY_PATH}/include)
#link_libraries("-L/${THIRDPARTY_PATH}/lib/ -lPQP")
rosbuild_add_executable(test_collision_core test/test_collision_core.cpp)
target_link_libraries(test_collision_core assimp)

rosbuild_add_gtest(test_collision_space_ccd test/test_collision_space_ccd.cpp)
target_link_libraries(test_collision_space_ccd collision_space_ccd)
target_link_libraries(test_collision_space_ccd assimp)