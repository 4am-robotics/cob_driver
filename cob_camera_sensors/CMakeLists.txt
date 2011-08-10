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
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/common/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rospack_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rospack_add_boost_directories()
#rospack_link_boost(${PROJECT_NAME} thread)
#rospack_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# add project libraries
rosbuild_add_library(cob_camera_sensors	common/src/AVTPikeCam.cpp
					common/src/AbstractColorCamera.cpp
					common/src/AbstractRangeImagingSensor.cpp
					common/src/VirtualColorCam.cpp
					common/src/VirtualRangeCam.cpp
					common/src/Swissranger.cpp)
rosbuild_add_library(cob_kinect_image_flip ros/src/kinect_image_flip.cpp)
add_custom_command(TARGET cob_kinect_image_flip POST_BUILD COMMAND mkdir -p ${PROJECT_SOURCE_DIR}/ros/lib/)
add_custom_command(TARGET cob_kinect_image_flip POST_BUILD COMMAND mv ${LIBRARY_OUTPUT_PATH}/libcob_kinect_image_flip.so ${PROJECT_SOURCE_DIR}/ros/lib/)
									
# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)


# add project executable
#rosbuild_add_executable(tof_camera ros/src/tof_camera.cpp)
#rosbuild_add_executable(tof_camera_viewer ros/src/tof_camera_viewer.cpp)
#rosbuild_add_executable(color_camera ros/src/color_camera.cpp)
#rosbuild_add_executable(all_cameras ros/src/all_cameras.cpp)
#rosbuild_add_executable(all_camera_viewer ros/src/all_camera_viewer.cpp)
#rosbuild_add_executable(undistort_tof ros/src/undistort_tof.cpp)

# add include search paths
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag
rosbuild_add_compile_flags(cob_camera_sensors -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(tof_camera -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(tof_camera_viewer -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(color_camera -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(all_cameras -D__LINUX__ -D__COB_ROS__)
#rosbuild_add_compile_flags(all_camera_viewer -D__LINUX__ -D__COB_ROS__ -D__ROS_1_1__)

# link libraries
target_link_libraries(cob_camera_sensors mesasr dc1394)
#target_link_libraries(tof_camera cob_camera_sensors usb)
#target_link_libraries(tof_camera_viewer cob_camera_sensors)
#target_link_libraries(color_camera cob_camera_sensors)
#target_link_libraries(all_cameras cob_camera_sensors usb)
