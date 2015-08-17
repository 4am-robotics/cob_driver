cmake_minimum_required(VERSION 2.8.3)
project(cob_head_axis)

find_package(catkin REQUIRED COMPONENTS actionlib cob_canopen_motor cob_generic_can cob_srvs cob_utilities control_msgs diagnostic_msgs roscpp sensor_msgs std_srvs urdf)

catkin_package()

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp common/src/ElmoCtrl.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/dummy_head.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
