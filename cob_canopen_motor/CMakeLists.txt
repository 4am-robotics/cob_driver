cmake_minimum_required(VERSION 3.0.2)
project(cob_canopen_motor)

find_package(catkin REQUIRED COMPONENTS cob_generic_can cob_utilities roscpp)

catkin_package(
  CATKIN_DEPENDS cob_generic_can cob_utilities roscpp
  INCLUDE_DIRS common/include
  LIBRARIES ${PROJECT_NAME}_harmonica
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_harmonica common/src/CanDriveHarmonica.cpp common/src/ElmoRecorder.cpp)

### INSTALL ###
install(TARGETS ${PROJECT_NAME}_harmonica
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
