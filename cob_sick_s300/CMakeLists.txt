
cmake_minimum_required(VERSION 2.8.3)
project(cob_sick_s300)

find_package(catkin REQUIRED COMPONENTS roscpp rostest sensor_msgs message_filters diagnostic_msgs tf laser_geometry)


include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp common/src/ScannerSickS300.cpp common/src/SerialIO.cpp)
add_executable(cob_scan_filter ros/src/cob_scan_filter.cpp)



# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rostest sensor_msgs message_filters diagnostic_msgs tf laser_geometry
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)