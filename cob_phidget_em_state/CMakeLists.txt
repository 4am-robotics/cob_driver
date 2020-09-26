cmake_minimum_required(VERSION 3.0.2)
project(cob_phidget_em_state)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_install_python(PROGRAMS scripts/em_state_phidget.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)
