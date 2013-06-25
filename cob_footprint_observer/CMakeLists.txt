cmake_minimum_required(VERSION 2.8.3)
project(cob_footprint_observer)

find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs message_generation)

add_service_files(
  FILES
  GetFootprint.srv 
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(footprint_observer src/cob_footprint_observer.cpp)
add_dependencies(footprint_observer ${PROJECT_NAME}_gencpp)
target_link_libraries(footprint_observer ${catkin_LIBRARIES} ${Boost_LIBRARIES})


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp tf std_msgs
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS include
    LIBRARIES # TODO
)
