
cmake_minimum_required(VERSION 2.8.3)
project(cob_relayboard)

find_package(catkin REQUIRED COMPONENTS roscpp rostest std_msgs pr2_msgs cob_utilities message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  DIRECTORY msg
  FILES EmergencyStopState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime roscpp rostest std_msgs cob_utilities pr2_msgs
    INCLUDE_DIRS common/include
    LIBRARIES SerialIO ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(SerialIO common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(SerialIO ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} SerialIO)

add_executable(cob_relayboard_node ros/src/cob_relayboard_node.cpp)
target_link_libraries(cob_relayboard_node ${PROJECT_NAME})
target_link_libraries(cob_relayboard_node ${catkin_LIBRARIES})
add_dependencies(cob_relayboard_node ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_relayboard_node SerialIO ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/relayboard_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

