##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(cob_light_sim)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build roscpp cob_light)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} include/cob_light_sim)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
  CATKIN_DEPENDS cob_light)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
rosbuild_prepare_qt4(QtCore QtGui QtOpenGL) # Add the appropriate components to the component list here
set(QT_USE_QTOPENGL TRUE)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui) 
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/cob_light_sim/*.hpp)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(cob_light_sim ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(cob_light_sim ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
install(TARGETS cob_light_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

