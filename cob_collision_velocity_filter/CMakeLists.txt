
cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_velocity_filter)

find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure nav_msgs tf cob_footprint_observer visualization_msgs)

generate_dynamic_reconfigure_options(cfg/DynParam.cfg)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)


add_executable(collision_velocity_filter
                        src/cob_collision_velocity_filter.cpp
                        src/velocity_limited_marker.cpp
                        )



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rospy dynamic_reconfigure move_base_msgs nav_msgs tf cob_footprint_observer visualization_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)