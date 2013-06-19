
cmake_minimum_required(VERSION 2.8.3)
project(cob_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS roscpp cob_srvs brics_actuator sensor_msgs actionlib control_msgs orocos_kdl)



add_service_files(
  FILES
  SetFloat.srv 
)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)


add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp common/src/genericArmCtrl.cpp common/src/RefValJS_PTP.cpp common/src/RefValJS_PTP_Trajectory.cpp common/src/TimeStamp.cpp)
add_executable(cob_simulation_tester ros/src/cob_simulation_tester.cpp)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

generate_messages(
  DEPENDENCIES geometry_msgs std_msgs
)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp cob_srvs brics_actuator sensor_msgs pr2_controllers_msgs actionlib control_msgs orocos_kdl
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)