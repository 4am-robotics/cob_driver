cmake_minimum_required(VERSION 2.8.3)
project(cob_head_axis)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cob_srvs actionlib cob_generic_can cob_canopen_motor cob_utilities urdf libntcan diagnostic_msgs control_msgs)




include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp common/src/ElmoCtrl.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rostest sensor_msgs cob_srvs actionlib cob_generic_can cob_canopen_motor cob_utilities urdf libntcan diagnostic_msgs control_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
