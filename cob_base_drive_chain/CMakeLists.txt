cmake_minimum_required(VERSION 2.8.3)
project(cob_base_drive_chain)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs message_generation diagnostic_msgs cob_srvs cob_generic_can cob_canopen_motor cob_utilities libpcan libntcan control_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  ElmoRecorderConfig.srv 
  ElmoRecorderReadout.srv 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp control_msgs cob_generic_can cob_canopen_motor cob_utilities libpcan  std_msgs geometry_msgs sensor_msgs diagnostic_msgs cob_srvs message_runtime
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

# add include search paths
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


# add project libs
add_library(${PROJECT_NAME} common/src/CanCtrlPltfCOb3.cpp)

# add executable
add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_sim_node ros/src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME}_sim_node PROPERTIES COMPILE_FLAGS "-D__SIM__")

target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_sim_node ${PROJECT_NAME} )

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_sim_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_sim_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
