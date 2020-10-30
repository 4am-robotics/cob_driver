cmake_minimum_required(VERSION 3.0.2)
project(cob_base_drive_chain)

find_package(catkin REQUIRED COMPONENTS cob_canopen_motor cob_generic_can cob_utilities control_msgs diagnostic_msgs message_generation roscpp sensor_msgs std_msgs std_srvs)

### Message Generatioin ###
add_service_files(
  FILES
  ElmoRecorderConfig.srv
  ElmoRecorderReadout.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
    CATKIN_DEPENDS control_msgs diagnostic_msgs message_runtime roscpp sensor_msgs std_msgs std_srvs
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/CanCtrlPltfCOb3.cpp)

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_sim_node ros/src/${PROJECT_NAME}.cpp)
set_target_properties(${PROJECT_NAME}_sim_node PROPERTIES COMPILE_FLAGS "-D__SIM__")
add_dependencies(${PROJECT_NAME}_sim_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_sim_node ${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_sim_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
