#!/usr/bin/env python
#***************************************************************
#
# Copyright (c) 2010
#
# Fraunhofer Institute for Manufacturing Engineering	
# and Automation (IPA)
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Project name: care-o-bot
# ROS stack name: cob_driver
# ROS package name: cob_arm
#								
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#			
# Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# Date of creation: Jan 2010
# ToDo:
#
# +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fraunhofer Institute for Manufacturing 
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#****************************************************************

import roslib; roslib.load_manifest('cob_arm')
import rospy
import actionlib

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('simple_trajectory')
    client = actionlib.SimpleActionClient('arm_controller/joint_trajectory_action', JointTrajectoryAction)
    client.wait_for_server()
    
    # Fill in the goal
    goal = JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    
    # First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names=["arm_1_joint","arm_2_joint","arm_3_joint","arm_4_joint","arm_5_joint","arm_6_joint","arm_7_joint"]

    # here you specify a list of trajectory points
    point=JointTrajectoryPoint()
    point.positions=[0,0,0,0,0,0,0]
    point.velocities=[0,0,0,0,0,0,0]
    point.time_from_start=rospy.Duration(1)
    goal.trajectory.points.append(point)

    point=JointTrajectoryPoint()
    point.positions=[0,0.7,0,0.7,1.5,1.5,0]
    point.velocities=[0,0,0,0,0,0,0]
    point.time_from_start=rospy.Duration(5)
    goal.trajectory.points.append(point)

    point=JointTrajectoryPoint()
    point.positions=[0,-0.7,0,-0.7,1.5,1.5,0]
    point.velocities=[0,0,0,0,0,0,0]
    point.time_from_start=rospy.Duration(10)
    goal.trajectory.points.append(point)
    
    point=JointTrajectoryPoint()
    point.positions=[0,0,0,0,0,0,0]
    point.velocities=[0,0,0,0,0,0,0]
    point.time_from_start=rospy.Duration(14)
    goal.trajectory.points.append(point)

    # print goal for debugging reasons
    print "start moving on trajectory"
    #print goal

    client.send_goal(goal)
#    client.wait_for_result()
#    print "movement finished"
