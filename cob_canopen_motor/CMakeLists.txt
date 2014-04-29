cmake_minimum_required(VERSION 2.8.3)
project(cob_canopen_motor)

find_package(catkin REQUIRED COMPONENTS roscpp cob_generic_can cob_utilities)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS roscpp cob_generic_can cob_utilities
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}_harmonica
)

###########
## Build ##
###########
# add include search paths
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add project libs
add_library(${PROJECT_NAME}_harmonica common/src/CanDriveHarmonica.cpp common/src/ElmoRecorder.cpp)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_harmonica
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
