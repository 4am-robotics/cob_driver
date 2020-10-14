cmake_minimum_required(VERSION 3.0.2)
project(laser_scan_densifier)

# List C++ dependencies on ros packages
set(ROS_CXX_DEPENDENCIES
  nodelet
  roscpp
  sensor_msgs
)

# Find catkin and all required ROS components
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# Set include directories
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare info that other packages need to import library generated here
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

#Create library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#Create nodelet
add_library(${PROJECT_NAME}_nodelet src/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME})

#Create node
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} )

#Install library
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_nodelet
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#Install library includes
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

#Install node
install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

#Install nodelet description
install(FILES ${PROJECT_NAME}_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )
