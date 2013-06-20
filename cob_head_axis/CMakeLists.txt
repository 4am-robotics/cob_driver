cmake_minimum_required(VERSION 2.8.3)
project(cob_head_axis)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cob_srvs actionlib cob_generic_can cob_canopen_motor cob_utilities urdf libntcan diagnostic_msgs control_msgs)




INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp common/src/ElmoCtrl.cpp)



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rostest sensor_msgs cob_srvs actionlib cob_generic_can cob_canopen_motor cob_utilities urdf libntcan diagnostic_msgs pr2_controllers_msgs control_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
