
cmake_minimum_required(VERSION 2.8.3)
project(cob_utilities)

find_package(catkin REQUIRED COMPONENTS roscpp)


# add include search paths
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common/include)

# add project libs
add_library(${PROJECT_NAME} common/src/IniFile.cpp common/src/MathSup.cpp common/src/StrUtil.cpp common/src/TimeStamp.cpp)

# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS roscpp
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}
)
