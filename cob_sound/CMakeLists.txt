
cmake_minimum_required(VERSION 2.8.3)
project(cob_sound)

find_package(catkin REQUIRED COMPONENTS rospy roscpp sound_play actionlib std_msgs cob_srvs diagnostic_msgs)



add_service_files(
  FILES
  SayText.srv
)
add_action_files(
   FILES
   Say.action 	
)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


add_executable(sound ros/src/sound.cpp)

target_link_libraries(sound ${Boost_LIBRARIES})


generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rospy roscpp sound_play actionlib std_msgs cob_srvs diagnostic_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)