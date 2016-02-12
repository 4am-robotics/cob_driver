cmake_minimum_required(VERSION 2.8.3)
project(cob_elmo_homing)

find_package(catkin REQUIRED COMPONENTS
  canopen_402
  class_loader
)
catkin_package()


include_directories(${catkin_INCLUDE_DIRS})

add_library(cob_elmo_homing_plugin src/plugin.cpp)
target_link_libraries(cob_elmo_homing_plugin
   ${catkin_LIBRARIES}
)

install(TARGETS cob_elmo_homing_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES canopen_402_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
