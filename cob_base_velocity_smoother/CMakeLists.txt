cmake_minimum_required(VERSION 2.8.3)
project(cob_base_velocity_smoother)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp geometry_msgs)


include_directories(include/cob_base_velocity_smoother ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(cob_base_velocity_smoother src/cob_base_velocity_smoother.cpp)
target_link_libraries(cob_base_velocity_smoother ${catkin_LIBRARIES})


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS std_msgs roscpp geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
