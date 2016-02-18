cmake_minimum_required(VERSION 2.8.3)
project(cob_bms_driver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  pluginlib
  roscpp
  roslaunch
  socketcan_interface
  std_msgs
)

catkin_package()

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(bms_driver_node src/cob_bms_driver_node.cpp)
add_dependencies(bms_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(bms_driver_node ${catkin_LIBRARIES})

#############
## Install ##
#############
install(TARGETS bms_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
roslaunch_add_file_check(launch)
