cmake_minimum_required(VERSION 2.8.3)
project(cob_cam3d_throttle)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pluginlib message_filters nodelet)


add_library(cam3d_throttle src/cam3d_throttle.cpp)



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp sensor_msgs pluginlib message_filters nodelet
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES cam3d_throttle
)
