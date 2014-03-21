cmake_minimum_required(VERSION 2.8.3)
project(cob_hokuyo)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs control_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp rostest sensor_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(cob_hokuyo_filter ros/src/cob_hokuyo_filter.cpp)
target_link_libraries(cob_hokuyo_filter ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_hokuyo_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
