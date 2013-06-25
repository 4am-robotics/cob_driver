cmake_minimum_required(VERSION 2.8.3)
project(cob_generic_can)

find_package(catkin REQUIRED COMPONENTS roscpp cob_utilities libpcan libntcan)

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add project libs
add_library(${PROJECT_NAME}_peaksysusb common/src/CanPeakSysUSB.cpp)
add_library(${PROJECT_NAME}_peaksys common/src/CanPeakSys.cpp)
add_library(${PROJECT_NAME}_esd common/src/CanESD.cpp)

# link libraries
target_link_libraries(${PROJECT_NAME}_peaksysusb cob_utilities ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_peaksys cob_utilities ${catkin_LIBRARIES})


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp cob_utilities libpcan
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}_peaksysusb ${PROJECT_NAME}_peaksys ${PROJECT_NAME}_esd 
)
