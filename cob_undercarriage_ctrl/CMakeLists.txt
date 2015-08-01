cmake_minimum_required(VERSION 2.8.3)
project(cob_undercarriage_ctrl)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs diagnostic_msgs tf cob_msgs cob_utilities diagnostic_updater control_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs diagnostic_msgs tf cob_msgs cob_utilities diagnostic_updater control_msgs INCLUDE_DIRS # TODO include
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS common/include
)

### BUILD ###
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/UndercarriageCtrlGeom.cpp)

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
