cmake_minimum_required(VERSION 2.8.3)
project(cob_camera_sensors)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pluginlib message_generation message_filters sensor_msgs nodelet)

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
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs pluginlib message_runtime message_filters nodelet
    INCLUDE_DIRS common/include
    LIBRARIES # TODO
)

install(DIRECTORY common/files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
)

install(DIRECTORY common/scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
)

install(PROGRAMS ros/scripts/set_camera_info.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY common/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)
