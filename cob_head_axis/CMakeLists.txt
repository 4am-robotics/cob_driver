cmake_minimum_required(VERSION 2.8.3)
project(cob_head_axis)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_srvs cob_srvs actionlib cob_generic_can cob_canopen_motor cob_utilities urdf libntcan diagnostic_msgs control_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp rostest sensor_msgs std_srvs cob_srvs actionlib cob_generic_can cob_canopen_motor cob_utilities urdf libntcan diagnostic_msgs control_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)



###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp common/src/ElmoCtrl.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS ros/src/dummy_head.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
