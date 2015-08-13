cmake_minimum_required(VERSION 2.8.3)
project(cob_voltage_control)

find_package(catkin REQUIRED COMPONENTS cob_msgs cob_phidgets dynamic_reconfigure roscpp std_msgs)

generate_dynamic_reconfigure_options(cfg/cob_voltage_control.cfg)

catkin_package()

### BUILD ###
include_directories(common/src ${catkin_INCLUDE_DIRS})

add_executable(cob_voltage_control ros/src/cob_voltage_control_ros.cpp)
add_dependencies(cob_voltage_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_voltage_control ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS cob_voltage_control
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS ros/src/record_current_and_voltage.py ros/src/savitzky.py ros/src/savitzky_golay.py ros/src/time_volt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
