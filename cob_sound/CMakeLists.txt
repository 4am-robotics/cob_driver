cmake_minimum_required(VERSION 3.0.2)
project(cob_sound)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs cob_srvs diagnostic_msgs message_generation roscpp std_msgs std_srvs visualization_msgs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libvlc REQUIRED libvlc)

add_action_files(DIRECTORY action FILES
   Play.action
   Say.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs cob_srvs diagnostic_msgs message_runtime roscpp std_msgs std_srvs visualization_msgs
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

catkin_install_python(PROGRAMS ros/src/test_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS fix_swift_for_precise.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
