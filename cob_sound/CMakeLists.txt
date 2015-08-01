
cmake_minimum_required(VERSION 2.8.3)
project(cob_sound)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs diagnostic_msgs visualization_msgs actionlib actionlib_msgs message_generation)

### Message Generation ###
add_service_files(DIRECTORY srv FILES
  SayText.srv
)
add_action_files(DIRECTORY action FILES
   Say.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs diagnostic_msgs visualization_msgs actionlib actionlib_msgs message_runtime sound_play
)

### BUILD ###
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(sound ros/src/sound.cpp)
add_dependencies(sound ${PROJECT_NAME}_gencpp)
target_link_libraries(sound ${Boost_LIBRARIES} ${catkin_LIBRARIES})


### INSTALL ###
install(TARGETS sound
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)

