cmake_minimum_required(VERSION 2.8.3)
project(cob_camera_sensors)

find_package(catkin REQUIRED COMPONENTS roscpp pluginlib message_generation message_filters sensor_msgs nodelet)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  AcquireCalibrationImages.srv 
  GetTOFImages.srv
)

## Generate added messages and services with any dependencies listed here

generate_messages(
   DEPENDENCIES std_msgs sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS #
    CATKIN_DEPENDS roscpp pluginlib message_runtime message_filters nodelet
    INCLUDE_DIRS common/include
    LIBRARIES # TODO
)


###########
## Build ##
###########
# add include search paths
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)


# add project executable
#add_executable(tof_camera ros/src/tof_camera.cpp)
#add_executable(tof_camera_viewer ros/src/tof_camera_viewer.cpp)
#add_executable(color_camera ros/src/color_camera.cpp)
#add_executable(all_cameras ros/src/all_cameras.cpp)
#add_executable(all_camera_viewer ros/src/all_camera_viewer.cpp)
#add_executable(undistort_tof ros/src/undistort_tof.cpp)

# add include search paths
#include_directories(${PROJECT_SOURCE_DIR}/common/include)

# add compile flag
#
# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(cob_camera_sensors -D__LINUX__ -D__COB_ROS__)
#
# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(tof_camera -D__LINUX__ -D__COB_ROS__)
#
# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(tof_camera_viewer -D__LINUX__ -D__COB_ROS__)
#
# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(color_camera -D__LINUX__ -D__COB_ROS__)
#
# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(all_cameras -D__LINUX__ -D__COB_ROS__)
#
# CATKIN_MIGRATION
# use link_directories() include_directories(), add_definitions(), target_link_libraries() or set_target_properties
# rosbuild_add_compile_flags(all_camera_viewer -D__LINUX__ -D__COB_ROS__ -D__ROS_1_1__)

# link libraries
#target_link_libraries(cob_camera_sensors mesasr dc1394 tinyxml)
#target_link_libraries(tof_camera cob_camera_sensors usb)
#target_link_libraries(tof_camera_viewer cob_camera_sensors)
#target_link_libraries(color_camera cob_camera_sensors)
#target_link_libraries(all_cameras cob_camera_sensors usb)

