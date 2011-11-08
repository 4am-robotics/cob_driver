cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(cob_ik_solver_kdl ros/src/ik_solver_kdl.cpp)
#rosbuild_add_executable(create_robot_manip_dae ros/src/create_robot_manip_dae.cpp)

rosbuild_add_library(cob3_arm_kinematics
  ros/src/ikfast_kinematics_plugin.cpp
)
#target_link_libraries(example ${PROJECT_NAME})

# rostest
#rosbuild_add_roslaunch_check(ros/launch/kdl_solver.launch ROBOT=cob3-1)
#rosbuild_add_roslaunch_check(ros/launch/kdl_solver.launch ROBOT=cob3-2)
#rosbuild_add_roslaunch_check(ros/launch/kdl_solver.launch ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(ros/launch/kdl_solver.launch ROBOT=cob3-4)
#rosbuild_add_rostest(ros/launch/kdl_solver.launch ROBOT=cob3-3)

#rosbuild_add_executable(cob_ik_regression_test ros/test/test_ik_regress.cpp)
#rosbuild_declare_test(cob_ik_regression_test)
#target_link_libraries(cob_ik_regression_test gtest)
#rosbuild_add_rostest(ros/test/test_cob_arm_ik.xml)
