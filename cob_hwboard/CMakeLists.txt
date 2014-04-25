cmake_minimum_required(VERSION 2.8.3)
project(cob_hwboard)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs diagnostic_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package( 
    CATKIN_DEPENDS rospy std_msgs diagnostic_msgs
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(PROGRAMS src/cob_hwboard.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)
