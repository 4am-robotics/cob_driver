cmake_minimum_required(VERSION 2.8.3)
project(cob_sick_s300)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs roscpp sensor_msgs std_msgs)

find_package(Boost REQUIRED COMPONENTS date_time thread)

catkin_package()

### BUILD ###
include_directories(common/include common/src ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(OODL_SickS300_SRC
  ${PROJECT_SOURCE_DIR}/common/src/Errors.cpp
  ${PROJECT_SOURCE_DIR}/common/src/Logger.cpp
 # ${PROJECT_SOURCE_DIR}/common/src/LaserScannerData.cpp
  ${PROJECT_SOURCE_DIR}/common/src/LaserScannerConfiguration.cpp
 # ${PROJECT_SOURCE_DIR}/common/src/LaserScannerDataWithIntensities.cpp
  ${PROJECT_SOURCE_DIR}/common/src/ScannerSickS300.cpp
  ${PROJECT_SOURCE_DIR}/common/src/SerialIO.cpp
)

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp ${OODL_SickS300_SRC})
add_executable(cob_scan_filter ros/src/cob_scan_filter.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(cob_scan_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
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
