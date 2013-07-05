cmake_minimum_required(VERSION 2.8.3)
project(cob_phidgets)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs libphidgets)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS roscpp rospy libphidgets std_msgs sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


###########
## Build ##
###########
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)

target_link_libraries(range_sensors ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS range_sensors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/simulated_range_sensors.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
