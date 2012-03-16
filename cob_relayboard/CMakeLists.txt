cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/common/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/ros/include)

#rosbuild_add_library(IniFile common/src/IniFile.cpp)
rosbuild_add_library(SerialIO common/src/SerialIO.cpp)
rosbuild_add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp)

target_link_libraries(${PROJECT_NAME} SerialIO)

#rosbuild_add_executable(neo_relayboard_test1 tests/test1.cpp)
	#target_link_libraries(neo_relayboard_test1 ${PROJECT_NAME})

rosbuild_add_executable(neo_relayboard_node ros/src/neo_relayboard_node.cpp ros/src/RelaisBoardNode.cpp)
target_link_libraries(neo_relayboard_node ${PROJECT_NAME})

rosbuild_add_executable(neo_srbdrive ros/src/neo_SRBDrive_node.cpp)
rosbuild_link_boost(neo_srbdrive thread)

rosbuild_add_executable(neo_srbgyro ros/src/neo_SRBGyro_node.cpp)
rosbuild_link_boost(neo_srbgyro thread)

rosbuild_add_executable(neo_srb_ir ros/src/neo_SRB_IR_node.cpp)
rosbuild_link_boost(neo_srb_ir thread)

rosbuild_add_executable(neo_srb_ir_tf_node ros/src/neo_tf_SRB_IR_node.cpp)
rosbuild_link_boost(neo_srb_ir_tf_node thread)

rosbuild_add_executable(neo_srb_us ros/src/neo_SRB_US_node.cpp)
rosbuild_link_boost(neo_srb_us thread)

rosbuild_add_executable(neo_srb_us_tf_node ros/src/neo_tf_SRB_US_node.cpp)
rosbuild_link_boost(neo_srb_us_tf_node thread)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
