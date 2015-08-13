cmake_minimum_required(VERSION 2.8.3)
project(cob_phidgets)

if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

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
  CATKIN_DEPENDS message_runtime std_msgs
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
  ros/src/phidget_sensors_node.cpp
  ros/src/phidget_manager.cpp
  common/src/phidget.cpp
  common/src/phidgetik.cpp
  ros/src/phidgetik_ros.cpp)
add_dependencies(phidget_sensors ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(phidget_sensors ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS range_sensors phidget_sensors device_name
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/src/simulated_range_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
