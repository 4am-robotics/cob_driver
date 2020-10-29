cmake_minimum_required(VERSION 3.0.2)
project(cob_undercarriage_ctrl)

find_package(catkin REQUIRED COMPONENTS cob_msgs cob_utilities control_msgs diagnostic_msgs diagnostic_updater geometry_msgs nav_msgs roscpp tf)

catkin_package(
  CATKIN_DEPENDS cob_msgs control_msgs diagnostic_msgs geometry_msgs nav_msgs roscpp tf
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/UndercarriageCtrlGeom.cpp)

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
