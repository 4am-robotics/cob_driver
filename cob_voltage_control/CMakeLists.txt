
cmake_minimum_required(VERSION 2.8.3)
project(cob_voltage_control)

find_package(catkin REQUIRED COMPONENTS roscpp libphidgets cob_relayboard rospy dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/cob_voltage_control.cfg)

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/src)

add_executable(cob_voltage_control ros/src/cob_voltage_control_ros.cpp)



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp libphidgets cob_relayboard rospy dynamic_reconfigure
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
