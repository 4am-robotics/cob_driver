cmake_minimum_required(VERSION 3.0.2)
project(cob_voltage_control)

find_package(catkin REQUIRED COMPONENTS cob_msgs cob_phidgets dynamic_reconfigure roscpp std_msgs)

generate_dynamic_reconfigure_options(cfg/${PROJECT_NAME}.cfg)

catkin_package(
  CATKIN_DEPENDS cob_msgs cob_phidgets dynamic_reconfigure roscpp std_msgs
)

### BUILD ###
include_directories(common/src ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}_ros.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

catkin_install_python(PROGRAMS ros/src/record_current_and_voltage.py ros/src/savitzky.py ros/src/savitzky_golay.py ros/src/time_volt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
