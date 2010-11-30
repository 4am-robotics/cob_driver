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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

rosbuild_add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp )
#rosbuild_add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp ros/src/sdhWrapper.cpp)
#rosbuild_add_executable(${PROJECT_NAME}_demo ros/src/${PROJECT_NAME}_demo.cpp ros/src/dsaboost.cpp)
#rosbuild_add_executable(${PROJECT_NAME}_driver ros/src/${PROJECT_NAME}_driver.cpp ros/src/dsaboost.cpp)

rosbuild_add_compile_flags(${PROJECT_NAME} -DOSNAME_LINUX)
#rosbuild_add_compile_flags(${PROJECT_NAME}_demo -DOSNAME_LINUX)
#rosbuild_add_compile_flags(${PROJECT_NAME}_driver -DOSNAME_LINUX)

rosbuild_add_compile_flags(${PROJECT_NAME} -DWITH_ESD_CAN)
#rosbuild_add_compile_flags(${PROJECT_NAME} -DWITH_PEAK_CAN)
#rosbuild_add_compile_flags(${PROJECT_NAME}_driver -DWITH_PEAK_CAN)

#target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan)
target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)
#target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)
#target_link_libraries(${PROJECT_NAME}_driver ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)
#${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so pcan)
