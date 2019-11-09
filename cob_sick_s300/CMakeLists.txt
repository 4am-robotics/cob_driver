cmake_minimum_required(VERSION 2.8.3)
project(cob_sick_s300)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs roscpp sensor_msgs std_msgs serial)

find_package(Boost REQUIRED COMPONENTS date_time thread)

catkin_package()

### BUILD ###
include_directories(
  common/include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  common/src/ScannerSickS300.cpp
  ros/src/${PROJECT_NAME}.cpp
)

add_executable(cob_scan_filter ros/src/cob_scan_filter.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(cob_scan_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if(WIN32)
  target_link_libraries(${PROJECT_NAME} ws2_32)
endif()
target_link_libraries(cob_scan_filter ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME} cob_scan_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
