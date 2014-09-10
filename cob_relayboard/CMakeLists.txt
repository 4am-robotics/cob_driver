
cmake_minimum_required(VERSION 2.8.3)
project(cob_relayboard)

find_package(catkin REQUIRED COMPONENTS roscpp rostest std_msgs cob_utilities cob_msgs)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp rostest std_msgs cob_utilities cob_msgs
    INCLUDE_DIRS common/include
    LIBRARIES SerialIO ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(SerialIO common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp)

add_dependencies(${PROJECT_NAME} cob_msgs_gencpp)
add_dependencies(SerialIO cob_msgs_gencpp)

target_link_libraries(${PROJECT_NAME} SerialIO)

add_executable(cob_relayboard_node ros/src/cob_relayboard_node.cpp)
target_link_libraries(cob_relayboard_node ${PROJECT_NAME})
target_link_libraries(cob_relayboard_node ${catkin_LIBRARIES})
add_dependencies(cob_relayboard_node cob_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_relayboard_node SerialIO ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/relayboard_sim.py ros/src/new_method.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

