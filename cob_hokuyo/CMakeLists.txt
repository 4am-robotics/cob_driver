cmake_minimum_required(VERSION 2.8.3)
project(cob_hokuyo)

find_package(catkin REQUIRED COMPONENTS roscpp rostest sensor_msgs pr2_controllers_msgs hokuyo_node)


include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(cob_hokuyo_filter ros/src/cob_hokuyo_filter.cpp)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rostest sensor_msgs pr2_controllers_msgs hokuyo_node
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)