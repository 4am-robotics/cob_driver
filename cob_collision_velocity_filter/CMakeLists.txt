
cmake_minimum_required(VERSION 2.8.3)
project(cob_collision_velocity_filter)

find_package(catkin REQUIRED COMPONENTS roscpp rospy dynamic_reconfigure move_base_msgs nav_msgs tf cob_footprint_observer visualization_msgs)

generate_dynamic_reconfigure_options(cfg/cob_collision_velocity_filter.cfg)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})



add_executable(collision_velocity_filter
                        src/cob_collision_velocity_filter.cpp
                        src/velocity_limited_marker.cpp
                        )

# add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


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