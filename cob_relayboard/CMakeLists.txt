cmake_minimum_required(VERSION 2.8.3)
project(cob_relayboard)

find_package(catkin REQUIRED COMPONENTS cob_msgs roscpp std_msgs)

catkin_package(
  INCLUDE_DIRS common/include
  LIBRARIES ${PROJECT_NAME}_SerialIO ${PROJECT_NAME}
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_SerialIO common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_SerialIO)

add_executable(cob_relayboard_node ros/src/cob_relayboard_node.cpp)
add_dependencies(cob_relayboard_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_relayboard_node ${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_relayboard_node ${PROJECT_NAME}_SerialIO ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ros/src/relayboard_sim.py ros/src/new_method.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
