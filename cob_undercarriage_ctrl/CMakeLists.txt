
cmake_minimum_required(VERSION 2.8.3)
project(cob_undercarriage_ctrl)

# define requierd packages for build this packages/
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs diagnostic_msgs tf cob_msgs cob_utilities diagnostic_updater control_msgs cob_omni_drive_controller)


###################################
## catkin specific configuration ##
###################################
# provide lib/packages which are requeird for the use of this package
catkin_package(
    CATKIN_DEPENDS roscpp nav_msgs geometry_msgs diagnostic_msgs tf cob_msgs cob_utilities diagnostic_updater
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


###########
## Build ##
###########
# add include search paths
include_directories(common/include ${catkin_INCLUDE_DIRS})

# add project - libs create lib for cob_undercarriage_ctrl_old
add_library(${PROJECT_NAME}_old common/src/UndercarriageCtrlGeom.cpp)

# add executable for cob_undercarriage_ctrl_node and cob_undercarriage_ctrl_node_old
add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
add_executable(${PROJECT_NAME}_node_old ros/src/${PROJECT_NAME}_old.cpp)

# link libraries for executable cob_undercarriage_ctrl_node
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
# link libraries for executable cob_undercarriage_ctrl_node_old
target_link_libraries(${PROJECT_NAME}_node_old
  ${PROJECT_NAME}_old
  ${catkin_LIBRARIES}
  yaml-cpp
)


#target_link_libraries(${PROJECT_NAME}_node yaml-cpp)
#target_link_libraries(${PROJECT_NAME}_node_old yaml-cpp)

#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#Geom Controller for one-wheel-COB3_5
#add_library(${PROJECT_NAME}_Trike common/src/UndercarriageCtrlGeomTrike.cpp)
#add_executable(${PROJECT_NAME}_Trike_node ros/src/${PROJECT_NAME}.cpp)
#target_link_libraries(${PROJECT_NAME}_Trike_node ${PROJECT_NAME}_Trike)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node #${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node_old ${PROJECT_NAME}_old
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

