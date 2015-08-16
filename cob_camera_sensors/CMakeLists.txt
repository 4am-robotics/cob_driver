cmake_minimum_required(VERSION 2.8.3)
project(cob_camera_sensors)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_vision_utils cv_bridge image_transport message_filters message_generation polled_camera roscpp sensor_msgs)

find_package(Boost REQUIRED COMPONENTS filesystem thread)

find_package(OpenCV REQUIRED)

find_package(TinyXML REQUIRED)

### Message Generation ###
add_service_files(
  FILES
  AcquireCalibrationImages.srv
  GetTOFImages.srv
)

generate_messages(
   DEPENDENCIES sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS cmake_modules cob_vision_utils message_runtime roscpp sensor_msgs
  DEPENDS Boost OpenCV
  INCLUDE_DIRS common/include
)

### INSTALL ###
install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES common/scripts/generateYAMLparameterFile.m
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common/scripts
)

install(PROGRAMS ros/scripts/set_camera_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
