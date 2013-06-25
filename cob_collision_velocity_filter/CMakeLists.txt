
cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_velocity_filter)

find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure nav_msgs tf cob_footprint_observer visualization_msgs)

generate_dynamic_reconfigure_options(cfg/DynParam.cfg)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(collision_velocity_filter
                        src/cob_collision_velocity_filter.cpp
                        src/velocity_limited_marker.cpp
                        )

target_link_libraries(collision_velocity_filter ${catkin_LIBRARIES})
add_dependencies(collision_velocity_filter ${PROJECT_NAME}_gencpp)




# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rospy dynamic_reconfigure move_base_msgs nav_msgs tf cob_footprint_observer visualization_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)