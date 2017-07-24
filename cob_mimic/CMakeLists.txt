cmake_minimum_required(VERSION 2.8.3)
project(cob_mimic)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation roscpp roslib)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem thread)
pkg_check_modules(libvlc REQUIRED libvlc)

### Message Generation ###
add_service_files(DIRECTORY srv FILES
  SetMimic.srv
)

add_action_files(DIRECTORY action FILES
  SetMimic.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

### BUILD ###
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(mimic src/mimic_node.cpp)
add_dependencies(mimic ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mimic ${catkin_LIBRARIES} ${libvlc_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS mimic
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/test_mimic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(DIRECTORY common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
