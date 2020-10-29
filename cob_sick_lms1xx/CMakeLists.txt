cmake_minimum_required(VERSION 3.0.2)
project(cob_sick_lms1xx)

find_package(catkin REQUIRED COMPONENTS diagnostic_msgs roscpp sensor_msgs)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs roscpp sensor_msgs
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(lms1xx common/src/lms1xx.cpp)

add_executable(lms1xx_test common/src/test.cpp)
add_executable(lms100 ros/src/lms1xx_node.cpp)
add_executable(set_config ros/src/set_config.cpp)

add_dependencies(lms100 ${catkin_EXPORTED_TARGETS})

target_link_libraries(lms1xx_test lms1xx ${catkin_LIBRARIES})
target_link_libraries(lms100 lms1xx ${catkin_LIBRARIES})
target_link_libraries(set_config lms1xx ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS lms100 lms1xx lms1xx_test set_config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
