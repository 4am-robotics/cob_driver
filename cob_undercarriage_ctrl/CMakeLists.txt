
cmake_minimum_required(VERSION 2.8.3)
project(cob_undercarriage_ctrl)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf cob_relayboard cob_utilities diagnostic_updater control_msgs)


###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf cob_relayboard cob_utilities diagnostic_updater
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


###########
## Build ##
###########
# add include search paths
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
# add project libs
add_library(${PROJECT_NAME} common/src/UndercarriageCtrlGeom.cpp)
# add executable
add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
# link libraries
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
#Geom Controller for one-wheel-COB3_5
#add_library(${PROJECT_NAME}_Trike common/src/UndercarriageCtrlGeomTrike.cpp)
#add_executable(${PROJECT_NAME}_Trike_node ros/src/${PROJECT_NAME}.cpp)
#target_link_libraries(${PROJECT_NAME}_Trike_node ${PROJECT_NAME}_Trike)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
