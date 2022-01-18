cmake_minimum_required(VERSION 3.0.2)
project(cob_light)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  diagnostic_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

### Message Generation ###
add_message_files(
  FILES
  ColorRGBAArray.msg
  LightMode.msg
  LightModes.msg
  Sequence.msg
)

add_service_files(
  FILES
  SetLightMode.srv
  StopLightMode.srv
)

add_action_files(
  FILES
  SetLightMode.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs sensor_msgs std_msgs visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs diagnostic_msgs message_runtime roscpp sensor_msgs std_msgs visualization_msgs
)

### BUILD ###
#pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
#pkg_check_modules(GTHREAD REQUIRED gthread-2.0)

add_compile_options(-Wno-narrowing)

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
set(HEADERS
    ${PROJECT_SOURCE_DIR}/common/include/breathColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/breathMode.h
    ${PROJECT_SOURCE_DIR}/common/include/circleColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/colorUtils.h
    ${PROJECT_SOURCE_DIR}/common/include/concurrentQueue.h
    ${PROJECT_SOURCE_DIR}/common/include/fadeColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/flashMode.h
    ${PROJECT_SOURCE_DIR}/common/include/mode.h
    ${PROJECT_SOURCE_DIR}/common/include/modeExecutor.h
    ${PROJECT_SOURCE_DIR}/common/include/modeFactory.h
    ${PROJECT_SOURCE_DIR}/common/include/modeRunner.h
    ${PROJECT_SOURCE_DIR}/common/include/sequenceMode.h
    ${PROJECT_SOURCE_DIR}/common/include/serialIO.h
    ${PROJECT_SOURCE_DIR}/common/include/staticMode.h
    ${PROJECT_SOURCE_DIR}/common/include/sweepColorMode.h
    ${PROJECT_SOURCE_DIR}/common/include/distApproxMode.h
    ${PROJECT_SOURCE_DIR}/common/include/xmasMode.h
    ${PROJECT_SOURCE_DIR}/common/include/kitMode.h
    ${PROJECT_SOURCE_DIR}/common/include/turnIndicatorMode.h
    ${PROJECT_SOURCE_DIR}/ros/include/colorO.h
    ${PROJECT_SOURCE_DIR}/ros/include/colorOMarker.h
    ${PROJECT_SOURCE_DIR}/ros/include/colorOSim.h
    ${PROJECT_SOURCE_DIR}/ros/include/iColorO.h
    ${PROJECT_SOURCE_DIR}/ros/include/ms35.h
    ${PROJECT_SOURCE_DIR}/ros/include/stageprofi.h
)


add_executable(${PROJECT_NAME}
    common/src/modeExecutor.cpp
    common/src/modeFactory.cpp
    common/src/serialIO.cpp
    ros/src/${PROJECT_NAME}.cpp
    ros/src/colorO.cpp
    ros/src/colorOMarker.cpp
    ros/src/colorOSim.cpp
    ros/src/ms35.cpp
    ros/src/stageprofi.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  ros/scripts/test.py
  ros/scripts/test_sprofi_circle.py
  ros/scripts/test_sprofi_modes.py
  ros/scripts/test_sprofi_part.py
  ros/scripts/light_action_test.py
  ros/scripts/light_service_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
