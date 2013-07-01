
cmake_minimum_required(VERSION 2.8.3)
project(cob_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS eigen roscpp cob_srvs brics_actuator sensor_msgs actionlib control_msgs orocos_kdl)

#######################################
## Declare ROS messages and services ##
#######################################

add_service_files(
  FILES
  SetFloat.srv 
)

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS roscpp cob_srvs brics_actuator sensor_msgs pr2_controllers_msgs actionlib control_msgs orocos_kdl
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(common/include ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp common/src/genericArmCtrl.cpp common/src/RefValJS_PTP.cpp common/src/RefValJS_PTP_Trajectory.cpp common/src/TimeStamp.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)


add_executable(cob_simulation_tester ros/src/cob_simulation_tester.cpp)
add_dependencies(cob_simulation_tester ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(cob_simulation_tester ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
