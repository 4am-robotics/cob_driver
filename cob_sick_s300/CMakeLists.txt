cmake_minimum_required(VERSION 3.0.2)
project(cob_sick_s300)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs roscpp sensor_msgs std_msgs)

find_package(Boost REQUIRED COMPONENTS date_time thread)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs roscpp sensor_msgs std_msgs
)

### BUILD ###
include_directories(
  common/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  common/src/ScannerSickS300.cpp
  common/src/SerialIO.cpp
  ros/src/${PROJECT_NAME}.cpp
)

add_executable(cob_scan_filter ros/src/cob_scan_filter.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(cob_scan_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(cob_scan_filter ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_scan_filter ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
