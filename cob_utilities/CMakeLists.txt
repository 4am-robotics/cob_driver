
cmake_minimum_required(VERSION 2.8.3)
project(cob_utilities)

find_package(catkin REQUIRED COMPONENTS roscpp)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########
# add include search paths
include_directories(${catkin_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add project libs
add_library(${PROJECT_NAME} common/src/IniFile.cpp common/src/MathSup.cpp common/src/StrUtil.cpp common/src/TimeStamp.cpp)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
