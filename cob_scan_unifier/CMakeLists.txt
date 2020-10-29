cmake_minimum_required(VERSION 3.0.2)
project(cob_scan_unifier)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_unifier_node src/scan_unifier_node.cpp)
target_link_libraries(scan_unifier_node ${catkin_LIBRARIES})
add_dependencies(scan_unifier_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS scan_unifier_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

