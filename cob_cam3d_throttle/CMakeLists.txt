cmake_minimum_required(VERSION 2.8.3)
project(cob_cam3d_throttle)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pluginlib message_filters nodelet)





###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs pluginlib message_filters nodelet sensor_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES cam3d_throttle
)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

add_library(cam3d_throttle src/cam3d_throttle.cpp)
target_link_libraries(cam3d_throttle ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cam3d_throttle
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
