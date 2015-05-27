
cmake_minimum_required(VERSION 2.8.3)
project(cob_sound)

find_package(catkin REQUIRED COMPONENTS message_generation rospy roscpp actionlib actionlib_msgs std_msgs std_srvs diagnostic_msgs visualization_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(DIRECTORY srv FILES
  SayText.srv
)
add_action_files(DIRECTORY action FILES
   Say.action 	
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS rospy roscpp actionlib actionlib_msgs std_msgs std_srvs diagnostic_msgs visualization_msgs message_runtime # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(sound ros/src/sound.cpp)

target_link_libraries(sound ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(sound ${PROJECT_NAME}_gencpp)
#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS sound
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)

