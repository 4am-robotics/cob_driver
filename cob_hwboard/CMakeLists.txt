cmake_minimum_required(VERSION 2.8.3)
project(cob_hwboard)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs diagnostic_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS rospy std_msgs diagnostic_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)