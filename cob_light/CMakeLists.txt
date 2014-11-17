cmake_minimum_required(VERSION 2.8.3)
project(cob_light)


find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs geometry_msgs visualization_msgs actionlib actionlib_msgs diagnostic_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS thread)


#######################################
## Declare ROS messages and services ##
#######################################
add_message_files(
  FILES
  LightMode.msg
)

add_service_files(
  FILES
  SetLightMode.srv
)

add_action_files(
  FILES
  SetLightMode.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime rospy roscpp std_msgs visualization_msgs actionlib diagnostic_msgs actionlib_msgs geometry_msgs
    INCLUDE_DIRS common/include
    LIBRARIES # TODO
)

###########
## Build ##
###########

#pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
#pkg_check_modules(GTHREAD REQUIRED gthread-2.0)

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})



add_executable(cob_light ros/src/cob_light.cpp ros/src/colorO.cpp ros/src/colorOSim.cpp ros/src/ms35.cpp ros/src/stageprofi.cpp
        common/src/serialIO.cpp common/src/modeExecutor.cpp common/src/modeFactory.cpp)
add_dependencies(cob_light ${PROJECT_NAME}_gencpp)

target_link_libraries(cob_light ${Boost_LIBRARIES})
target_link_libraries(cob_light ${catkin_LIBRARIES})

###########
## Test ##
###########
# rostest
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-1)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-2)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-4)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_light
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ros/scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
