cmake_minimum_required(VERSION 2.8.3)
project(cob_phidgets)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs libphidgets)


include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)

target_link_libraries(range_sensors ${catkin_LIBRARIES})

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
