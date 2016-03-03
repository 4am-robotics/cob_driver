cmake_minimum_required(VERSION 2.8.3)
project(cob_sound)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib cob_srvs diagnostic_msgs message_generation roscpp std_msgs std_srvs visualization_msgs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libvlc REQUIRED libvlc)

add_action_files(DIRECTORY action FILES
   Say.action
   Play.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

### BUILD ###
include_directories(${catkin_INCLUDE_DIRS})

add_executable(sound ros/src/sound.cpp)
add_dependencies(sound ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sound ${catkin_LIBRARIES} ${libvlc_LIBRARIES})

### INSTALL ###
install(TARGETS sound
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
