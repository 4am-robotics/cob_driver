cmake_minimum_required(VERSION 2.8.3)
project(cob_canopen_motor)

find_package(catkin REQUIRED COMPONENTS roscpp cob_generic_can cob_utilities)


include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add project libs
add_library(${PROJECT_NAME}_harmonica common/src/CanDriveHarmonica.cpp common/src/ElmoRecorder.cpp)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp cob_generic_can cob_utilities
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)