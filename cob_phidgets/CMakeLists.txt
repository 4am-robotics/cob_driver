cmake_minimum_required(VERSION 3.0.2)
project(cob_phidgets)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS libphidgets message_generation roscpp sensor_msgs std_msgs)

### Message Generation ###
add_message_files(
  FILES
  AnalogSensor.msg
  DigitalSensor.msg
)

add_service_files(
  FILES
  SetDataRate.srv
  SetDigitalSensor.srv
  SetTriggerValue.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
)

### BUILD ###
include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS})

add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)
add_dependencies(range_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(range_sensors ${catkin_LIBRARIES})

add_executable(device_name ros/src/set_device_name.cpp)
add_dependencies(device_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(device_name ${catkin_LIBRARIES})

add_executable(phidget_sensors
  common/src/phidget.cpp
  common/src/phidgetik.cpp
  ros/src/phidget_manager.cpp
  ros/src/phidget_sensors_node.cpp
  ros/src/phidgetik_ros.cpp)
add_dependencies(phidget_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(phidget_sensors ${catkin_LIBRARIES})

add_executable(phidget_sensors_sim ros/src/cob_phidgets_sim_node.cpp)
add_dependencies(phidget_sensors_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(phidget_sensors_sim ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS device_name phidget_sensors phidget_sensors_sim range_sensors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS ros/src/simulated_range_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
