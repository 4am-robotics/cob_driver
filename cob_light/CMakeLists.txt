cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
#pkg_check_modules(GTHREAD REQUIRED gthread-2.0)

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include)

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
rosbuild_add_executable(cob_light ros/src/cob_light.cpp ros/src/colorO.cpp ros/src/colorOSim.cpp common/src/serialIO.cpp common/src/modeExecutor.cpp common/src/modeFactory.cpp)
rosbuild_link_boost(cob_light thread)

# rostest
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-1)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-2)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-4)
