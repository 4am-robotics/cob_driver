cmake_minimum_required(VERSION 3.0.2)
project(cob_elmo_homing)

find_package(catkin REQUIRED COMPONENTS
  canopen_402
  pluginlib
  socketcan_interface
)
catkin_package()


include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_plugin src/plugin.cpp)
target_link_libraries(${PROJECT_NAME}_plugin
   ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES canopen_402_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
