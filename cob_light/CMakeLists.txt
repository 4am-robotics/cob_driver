cmake_minimum_required(VERSION 2.8.3)
project(cob_light)


find_package(catkin REQUIRED COMPONENTS rospy roscpp std_msgs visualization_msgs actionlib actionlib_msgs diagnostic_msgs)


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
#pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
#pkg_check_modules(GTHREAD REQUIRED gthread-2.0)

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include)


add_executable(cob_light ros/src/cob_light.cpp ros/src/colorO.cpp ros/src/colorOSim.cpp common/src/serialIO.cpp common/src/modeExecutor.cpp common/src/modeFactory.cpp)
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(cob_light ${Boost_LIBRARIES})
target_link_libraries(cob_light ${catkin_LIBRARIES})

# rostest
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-1)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-2)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-3)
#rosbuild_add_roslaunch_check(ros/launch/light.launch ROBOT=cob3-4)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs std_msgs
)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS rospy roscpp std_msgs visualization_msgs actionlib diagnostic_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
