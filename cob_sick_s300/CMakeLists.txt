
cmake_minimum_required(VERSION 2.8.3)
project(cob_sick_s300)

find_package(catkin REQUIRED COMPONENTS roscpp rostest sensor_msgs message_filters diagnostic_msgs tf laser_geometry)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS roscpp rostest sensor_msgs message_filters diagnostic_msgs tf laser_geometry
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########
# add include search paths
include_directories(common/include common/src ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

SET(OODL_SickS300_SRC
  ${PROJECT_SOURCE_DIR}/common/src/Errors.cpp
  ${PROJECT_SOURCE_DIR}/common/src/Logger.cpp
 # ${PROJECT_SOURCE_DIR}/common/src/LaserScannerData.cpp
  ${PROJECT_SOURCE_DIR}/common/src/LaserScannerConfiguration.cpp
 # ${PROJECT_SOURCE_DIR}/common/src/LaserScannerDataWithIntensities.cpp
  ${PROJECT_SOURCE_DIR}/common/src/SickS300.cpp
  ${PROJECT_SOURCE_DIR}/common/src/ScannerSickS300.cpp
  ${PROJECT_SOURCE_DIR}/common/src/SerialIO.cpp
)

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp ${OODL_SickS300_SRC})
add_executable(cob_scan_filter ros/src/cob_scan_filter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(cob_scan_filter ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

