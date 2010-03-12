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
#genmsg()
#uncomment if you have defined services
#gensrv()

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
					common/src/CameraSensorToolbox.cpp
					common/src/VirtualColorCam.cpp
					common/src/VirtualRangeCam.cpp
					common/src/Swissranger.cpp)
									
# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag
rosbuild_add_compile_flags(cob_camera_sensors -D__LINUX__ -D__COB_ROS__)


# add project executable
rosbuild_add_executable(tof_camera ros/src/tof_camera.cpp)
rosbuild_add_executable(tof_camera_viewer ros/src/tof_camera_viewer.cpp)
rosbuild_add_executable(color_camera ros/src/color_camera.cpp)

# add include search paths
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag
rosbuild_add_compile_flags(tof_camera -D__LINUX__ -D__COB_ROS__)
rosbuild_add_compile_flags(tof_camera_viewer -D__LINUX__ -D__COB_ROS__)
rosbuild_add_compile_flags(color_camera -D__LINUX__ -D__COB_ROS__)

# link libraries
target_link_libraries(cob_camera_sensors dc1394 cv cxcore highgui mesasr Wm4Foundation cob_camera_sensors cob_vision_utils tinyxml)
target_link_libraries(tof_camera dc1394 cv cxcore highgui mesasr Wm4Foundation cob_camera_sensors cob_vision_utils tinyxml)
target_link_libraries(tof_camera_viewer cv cxcore highgui mesasr Wm4Foundation cob_camera_sensors cob_vision_utils tinyxml)
target_link_libraries(color_camera dc1394 cv cxcore highgui mesasr Wm4Foundation cob_camera_sensors cob_vision_utils tinyxml)
