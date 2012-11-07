/****************************************************************
 *
 * Copyright (c) 2012 Brno University of Technology (BUT)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_navigation
 * ROS package name: cob_collision_velocity_filter
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Michal Spanel, email:spanel@fit.vutbr.cz
 *
 * Date of creation: November 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/
#include <velocity_limited_marker.h>

#include <tf/tf.h>


namespace cob_collision_velocity_filter
{

const double MARKER_RADIUS = 0.1;
const double MARKER_WIDTH = 13.0;


VelocityLimitedMarker::VelocityLimitedMarker()
{
    // a handle for this node, initialize node
    nh_ = ros::NodeHandle("~");

    // read parameters from parameter server
    nh_.param("marker_frame", base_frame_, std::string("/base_link"));
    nh_.param("marker_topic_name", topic_name_, std::string("velocity_limited_marker"));
    nh_.param("marker_lifetime", lifetime_, 1.0);
    nh_.param("marker_disabled", disabled_, false);

    // Create the publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_name_, 1);

    vx_last_ = 0.0;
    vy_last_ = 0.0;
    vtheta_last_ = 0.0;

    // Create the markers
    createMarkers();
}


VelocityLimitedMarker::~VelocityLimitedMarker()
{
}


void VelocityLimitedMarker::createMarkers()
{
    // Message template
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cob_velocity_limited_marker";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(lifetime_);
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = MARKER_RADIUS;
    marker.scale.y = MARKER_RADIUS;
    marker.scale.z = MARKER_RADIUS;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5; // adjust according to the velocity?

    // Create the disc like geometry for the markers
    std::vector<geometry_msgs::Point> circle1, circle2;
    geometry_msgs::Point v1, v2;

    static const int STEPS = 36;
    static const int FIRST = -4;
    static const int LAST = 4;
    for( int i = FIRST; i <= LAST; ++i )
    {
        float a = float(i) / float(STEPS) * M_PI * 2.0;
        
        v1.x = 0.5 * cos(a);
        v1.y = 0.5 * sin(a);
        v2.x = (1 + MARKER_WIDTH) * v1.x;
        v2.y = (1 + MARKER_WIDTH) * v1.y;

        circle1.push_back(v1);
        circle2.push_back(v2);
    }

    marker.points.clear();
    for( std::size_t i = 0; i < circle1.size(); ++i )
    {
        std::size_t i1 = i;
        std::size_t i2 = (i + 1) % circle1.size();

        marker.points.push_back(circle1[i1]);
        marker.points.push_back(circle2[i1]);
        marker.points.push_back(circle1[i2]);
        marker.points.push_back(circle2[i1]); 
        marker.points.push_back(circle2[i2]);
        marker.points.push_back(circle1[i2]);
    }

    // Particular messages for each axis
    x_pos_marker_ = marker;
    x_pos_marker_.id = 0;
    x_pos_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    x_neg_marker_ = marker;
    x_neg_marker_.id = 1;
    x_neg_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);

    y_pos_marker_ = marker;
    y_pos_marker_.id = 2;
    y_pos_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.5 * M_PI);

    y_neg_marker_ = marker;
    y_neg_marker_.id = 3;
    y_neg_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -0.5 * M_PI);
}

void VelocityLimitedMarker::publishMarkers( double vel_x_desired, 
                                            double vel_x_actual, 
                                            double vel_y_desired, 
                                            double vel_y_actual, 
                                            double vel_thetha_desired, 
                                            double vel_theta_actual)
{
    if( disabled_ )
    {
        return;
    }

    // Publish the markers
    double epsilon = 0.05;

    // acceleration
    double ax,ay,atheta;
    ax = vel_x_actual - vx_last_;
    ay = vel_y_actual - vy_last_;
    atheta = vel_theta_actual - vtheta_last_;
    vx_last_ = vel_x_actual;
    vy_last_ = vel_y_actual;
    vtheta_last_ = vel_theta_actual;

    // for x-axis
    ROS_INFO("vx_desired = %f, vx_actual = %f, ax = %f",vel_x_desired,vel_x_actual,ax);
    if( fabs(vel_x_desired - vel_x_actual) >= epsilon )
    {
        if (vel_x_desired >= 0.0 && ax <= 0.0)
        {
            x_pos_marker_.header.stamp = ros::Time::now();
        	marker_pub_.publish(x_pos_marker_);            
        }
        else if (vel_x_desired <= 0.0 && ax >= 0.0)
        {
            x_neg_marker_.header.stamp = ros::Time::now();
    	    marker_pub_.publish(x_neg_marker_);
        }
    }

    // for y-axis
    if( fabs(vel_y_desired - vel_y_actual) >= epsilon )
    {
        if (vel_y_desired >= 0.0 && ay <= 0.0)
        {
            y_pos_marker_.header.stamp = ros::Time::now();
        	marker_pub_.publish(y_pos_marker_);            
        }
        else if (vel_y_desired <= 0.0 && ay >= 0.0)
        {
            y_neg_marker_.header.stamp = ros::Time::now();
    	    marker_pub_.publish(y_neg_marker_);
        }
    }

/*
    // for theta-axis
    if( fabs(vel_theta_desired - vel_that_actual) >= epsilon )
    {
        if (vel_theta_desired >= 0.0)
        {
            theta_pos_marker_.header.stamp = ros::Time::now();
        	marker_pub_.publish(theta_pos_marker_);            
        }
        else
        {
            theta_neg_marker_.header.stamp = ros::Time::now();
    	    marker_pub_.publish(theta_neg_marker_);
        }
    }
*/
}


}

