cmake_minimum_required(VERSION 2.8.3)
project(cob_light)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib diagnostic_msgs message_generation roscpp sensor_msgs std_msgs visualization_msgs)

find_package(Boost REQUIRED COMPONENTS signals thread)

### Message Generation ###
add_message_files(
  FILES
  LightMode.msg
  LightModes.msg
  Sequence.msg
  ColorRGBAArray.msg
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
  DEPENDENCIES actionlib_msgs std_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime std_msgs sensor_msgs
)

### BUILD ###
#pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)
#pkg_check_modules(GTHREAD REQUIRED gthread-2.0)

include_directories(${PROJECT_SOURCE_DIR}/ros/include ${PROJECT_SOURCE_DIR}/common/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
set(HEADERS
    ${PROJECT_SOURCE_DIR}/ros/include/colorO.h
    ${PROJECT_SOURCE_DIR}/ros/include/colorOSim.h
    ${PROJECT_SOURCE_DIR}/ros/include/iColorO.h
    ${PROJECT_SOURCE_DIR}/ros/include/ms35.h
    ${PROJECT_SOURCE_DIR}/ros/include/stageprofi.h
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
)


add_executable(cob_light ros/src/cob_light.cpp ros/src/colorO.cpp ros/src/colorOSim.cpp ros/src/ms35.cpp ros/src/stageprofi.cpp
                         common/src/serialIO.cpp common/src/modeExecutor.cpp common/src/modeFactory.cpp ${HEADERS})
add_dependencies(cob_light ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_light ${catkin_LIBRARIES} ${Boost_LIBRARIES})

### INSTALL ###
install(TARGETS cob_light
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS ros/scripts/test.py ros/scripts/test_sprofi_circle.py ros/scripts/test_sprofi_modes.py ros/scripts/test_sprofi_part.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/scripts
)
