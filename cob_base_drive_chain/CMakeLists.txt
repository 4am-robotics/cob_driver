cmake_minimum_required(VERSION 2.8.3)
project(cob_base_drive_chain)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs diagnostic_msgs cob_srvs cob_generic_can cob_canopen_motor cob_utilities libpcan libntcan control_msgs)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/common/lib)

add_service_files(
  FILES
  ElmoRecorderConfig.srv 
  ElmoRecorderReadout.srv 
)



# add include search paths
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


# add project libs
add_library(${PROJECT_NAME} common/src/CanCtrlPltfCOb3.cpp)

# add executable
add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_sim_node ros/src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} cob_canopen_motor_harmonica)
target_link_libraries(${PROJECT_NAME}_sim_node ${PROJECT_NAME} 
cob_canopen_motor_harmonica)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_sim_node ${catkin_LIBRARIES})

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
    DEPENDS roscpp sensor_msgs diagnostic_msgs cob_srvs cob_generic_can cob_canopen_motor cob_utilities libpcan
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}
)
