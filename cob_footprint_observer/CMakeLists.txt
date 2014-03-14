cmake_minimum_required(VERSION 2.8.3)
project(cob_footprint_observer)

find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs geometry_msgs message_generation)


if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()


#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  GetFootprint.srv 
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package( 
    CATKIN_DEPENDS roscpp tf std_msgs geometry_msgs message_runtime
    INCLUDE_DIRS include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(footprint_observer src/cob_footprint_observer.cpp)
add_dependencies(footprint_observer ${PROJECT_NAME}_gencpp)
target_link_libraries(footprint_observer ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS footprint_observer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)







