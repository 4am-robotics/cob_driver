cmake_minimum_required(VERSION 2.8.3)
project(cob_sick_lms1xx)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs)

include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(lms1xx common/src/lms1xx.cpp)

add_executable(lms1xx_test common/src/test.cpp)
add_executable(lms100 ros/src/lms1xx_node.cpp)
add_executable(set_config ros/src/set_config.cpp)

target_link_libraries(lms1xx_test lms1xx ${catkin_LIBRARIES})
target_link_libraries(lms100 lms1xx ${catkin_LIBRARIES})
target_link_libraries(set_config lms1xx ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS lms1xx lms1xx_test lms100 set_config
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
