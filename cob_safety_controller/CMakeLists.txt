cmake_minimum_required(VERSION 2.8.3)
project(cob_safety_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
    dynamic_reconfigure 
	nav_msgs
	visualization_msgs
	cob_srvs
	sick_flexisoft_client
)

# protected region additional user defined REQUIREMENTS on begin #
# protected region additional user defined REQUIREMENTS end #

generate_dynamic_reconfigure_options(cfg/safety_controller.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    common/src
  CATKIN_DEPENDS 
  	roscpp
    dynamic_reconfigure
	nav_msgs
	visualization_msgs
	cob_srvs
	sick_flexisoft_client
   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(common/include)
include_directories(common/src)
include_directories(${catkin_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(safety_controller ros/src/safety_controller_ros.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(safety_controller safety_controller_gencfg)
add_dependencies(safety_controller ${cob_safety_controller_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(safety_controller
  ${catkin_LIBRARIES}
)

# protected region additional user defined BUILD STATEMENTS on begin #
# protected region additional user defined BUILD STATEMENTS end #


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS safety_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# protected region user Cmake macros on begin #
# protected region user Cmake macros end #
