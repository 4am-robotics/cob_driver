
cmake_minimum_required(VERSION 2.8.3)
project(cob_undercarriage_ctrl)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf cob_relayboard cob_utilities diagnostic_updater pr2_controllers_msgs)


include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})


# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)
# add project libs
add_library(${PROJECT_NAME} common/src/UndercarriageCtrlGeom.cpp)
# add executable
add_executable(${PROJECT_NAME}_node ros/src/${PROJECT_NAME}.cpp)
# link libraries
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#Geom Controller for one-wheel-COB3_5
#add_library(${PROJECT_NAME}_Trike common/src/UndercarriageCtrlGeomTrike.cpp)
#add_executable(${PROJECT_NAME}_Trike_node ros/src/${PROJECT_NAME}.cpp)
#target_link_libraries(${PROJECT_NAME}_Trike_node ${PROJECT_NAME}_Trike)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs diagnostic_msgs tf cob_relayboard cob_utilities diagnostic_updater pr2_controllers_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)