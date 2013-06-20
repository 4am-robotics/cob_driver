
cmake_minimum_required(VERSION 2.8.3)
project(cob_relayboard)

find_package(catkin REQUIRED COMPONENTS roscpp rostest std_msgs cob_utilities)


add_message_files(
  FILES
  EmergencyStopState.msg
)


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

add_library(SerialIO common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp)

target_link_libraries(${PROJECT_NAME} SerialIO)

add_executable(cob_relayboard_node ros/src/cob_relayboard_node.cpp)
target_link_libraries(cob_relayboard_node ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp rostest std_msgs cob_utilities pr2_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)