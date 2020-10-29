cmake_minimum_required(VERSION 3.0.2)
project(cob_relayboard)

find_package(catkin REQUIRED COMPONENTS cob_msgs roscpp std_msgs)

catkin_package(
  CATKIN_DEPENDS cob_msgs roscpp std_msgs
  INCLUDE_DIRS common/include
  LIBRARIES ${PROJECT_NAME}_SerialIO ${PROJECT_NAME}
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_SerialIO common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_SerialIO)

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_SerialIO ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS ros/src/relayboard_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
