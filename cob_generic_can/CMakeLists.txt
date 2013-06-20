cmake_minimum_required(VERSION 2.8.3)
project(cob_generic_can)

find_package(catkin REQUIRED COMPONENTS roscpp cob_utilities libpcan libntcan)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add project libs
add_library(${PROJECT_NAME}_peaksysusb common/src/CanPeakSysUSB.cpp)
add_library(${PROJECT_NAME}_peaksys common/src/CanPeakSys.cpp)
add_library(${PROJECT_NAME}_esd common/src/CanESD.cpp)

# link libraries
target_link_libraries(${PROJECT_NAME}_peaksysusb pcan cob_utilities)
target_link_libraries(${PROJECT_NAME}_peaksys pcan cob_utilities)
target_link_libraries(${PROJECT_NAME}_esd ntcan cob_utilities)


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp cob_utilities libpcan libntcan
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)