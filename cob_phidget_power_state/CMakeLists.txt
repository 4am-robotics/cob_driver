cmake_minimum_required(VERSION 2.8.3)
project(cob_phidget_power_state)

find_package(catkin REQUIRED COMPONENTS
  cob_msgs
  cob_phidgets
)

catkin_package()

install(PROGRAMS scripts/power_state_phidget.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
