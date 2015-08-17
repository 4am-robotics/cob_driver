cmake_minimum_required(VERSION 2.8.3)
project(cob_mimic)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation)

### Message Generation ###
add_service_files(DIRECTORY srv FILES
  SetMimic.srv
)

add_action_files(DIRECTORY action FILES
  SetMimic.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

### INSTALL ###
install(PROGRAMS src/mimic_node.py src/test_mimic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

install(DIRECTORY common
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
