cmake_minimum_required(VERSION 2.8.3)
project(cob_phidgets)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs libphidgets message_generation)


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

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp rospy libphidgets std_msgs sensor_msgs message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


###########
## Build ##
###########
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)

target_link_libraries(range_sensors ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS range_sensors
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ros/src/simulated_range_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
