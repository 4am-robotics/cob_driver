cmake_minimum_required(VERSION 2.4.6)

if(CMAKE_COMPILER_IS_GNUCXX)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
   if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
        message(STATUS "C++11 activated.")
        add_definitions("-std=gnu++11")
   elseif(GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3)
        message(WARNING "C++0x activated. If you get any errors update to a compiler which fully supports C++11")
        add_definitions("-std=gnu++0x")
   else ()
        message(FATAL_ERROR "C++11 needed. Therefore a gcc compiler with a version higher than 4.3 is needed.")
   endif()
else(CMAKE_COMPILER_IS_GNUCXX)
   add_definitions("-std=c++0x")
endif(CMAKE_COMPILER_IS_GNUCXX)

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
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include)

rosbuild_add_executable(range_sensors ros/src/phidgets_range_sensors.cpp)
target_link_libraries(range_sensors phidget21)

rosbuild_add_executable(device_name ros/src/set_device_name.cpp)
target_link_libraries(device_name phidget21)

rosbuild_add_executable(phidget_sensors ros/src/phidget_sensors_node.cpp ros/src/phidget_manager.cpp 
						common/src/phidget.cpp common/src/phidgetik.cpp ros/src/phidgetik_ros.cpp)
target_link_libraries(phidget_sensors phidget21)

list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs" )