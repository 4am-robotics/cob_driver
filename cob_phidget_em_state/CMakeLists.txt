cmake_minimum_required(VERSION 2.8.3)
project(cob_phidget_em_state)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(PROGRAMS scripts/em_state_phidget.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
