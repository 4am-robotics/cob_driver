
cmake_minimum_required(VERSION 2.8.3)
project(cob_sound)

find_package(catkin REQUIRED COMPONENTS rospy roscpp actionlib actionlib_msgs std_msgs cob_srvs diagnostic_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  SayText.srv
)
add_action_files(
   FILES
   Say.action 	
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS rospy roscpp sound_play actionlib std_msgs cob_srvs diagnostic_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(sound ros/src/sound.cpp)

target_link_libraries(sound ${catkin_LIBRARIES} ${Boost_LIBRARIES})
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

## Mark cpp header files for installation
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)