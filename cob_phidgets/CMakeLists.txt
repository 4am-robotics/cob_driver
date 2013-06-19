cmake_minimum_required(VERSION 2.8.3)
project(cob_phidgets)

find_package(catkin REQUIRED COMPONENTS roscpp rospy libphidgets std_msgs sensor_msgs)


add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)

target_link_libraries(range_sensors phidget21)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rospy libphidgets std_msgs sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
