cmake_minimum_required(VERSION 3.0.2)
project(cob_utilities)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS common/include
  LIBRARIES ${PROJECT_NAME}
)

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/IniFile.cpp common/src/MathSup.cpp common/src/StrUtil.cpp common/src/TimeStamp.cpp)

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
