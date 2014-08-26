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
   DEPENDENCIES sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs pluginlib message_runtime message_filters nodelet
    INCLUDE_DIRS common/include
    LIBRARIES # TODO
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY common/scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common
)

install(PROGRAMS ros/scripts/set_camera_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

