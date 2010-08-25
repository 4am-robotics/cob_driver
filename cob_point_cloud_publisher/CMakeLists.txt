cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ros/include)


rosbuild_add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp)
rosbuild_add_library(nodelet_cpc_publisher ros/src/cob_colored_point_cloud_publisher.cpp)
rosbuild_add_executable(cpc_publisher ros/src/cpc_publisher_main.cpp)

target_link_libraries(cpc_publisher nodelet_cpc_publisher)

# add compile flag
rosbuild_add_compile_flags(${PROJECT_NAME} -D__LINUX__ -D__COB_ROS__)
rosbuild_add_compile_flags(nodelet_cpc_publisher -D__LINUX__ -D__COB_ROS__)
rosbuild_add_compile_flags(cpc_publisher -D__LINUX__ -D__COB_ROS__)
