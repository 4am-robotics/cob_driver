cmake_minimum_required(VERSION 2.8.3)
project(cob_base_velocity_smoother)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp geometry_msgs)


if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()


###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS std_msgs roscpp geometry_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
)


###########
## Build ##
###########

include_directories(include/cob_base_velocity_smoother ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/cob_base_velocity_smoother.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS cob_base_velocity_smoother
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

