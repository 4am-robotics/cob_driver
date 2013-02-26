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

const double DEFAULT_LIFETIME   = 1.0;
const double DEFAULT_Z_POSITION = 0.25;

const double MARKER_SCALE_DIR   = 1.0;
const double MARKER_RADIUS_DIR  = 0.5;
const double MARKER_WIDTH_DIR   = 0.2;

const double MARKER_SCALE_ROT   = 1.0;
const double MARKER_RADIUS_ROT  = 0.6;
//const double MARKER_WIDTH_ROT   = 0.065;
const double MARKER_WIDTH_ROT   = 0.15;

//const double MAX_VELOCITY       = 0.2;
const double MAX_VELOCITY       = 0.25;
const double MIN_VELOCITY       = 0.01;
const double VELOCITY_COEFF     = 1.0 / (MAX_VELOCITY - MIN_VELOCITY);

const float MAX_COLOR[4]        = { 1.0f, 0.0f, 0.0f, 1.0f };
const float MIN_COLOR[4]        = { 1.0f, 0.8f, 0.0f, 0.2f };


VelocityLimitedMarker::VelocityLimitedMarker()
{
    // a handle for this node, initialize node
    nh_ = ros::NodeHandle("~");

    // read parameters from parameter server
    nh_.param("marker_frame", base_frame_, std::string("/base_link"));
    nh_.param("marker_topic_name", topic_name_, std::string("velocity_limited_marker"));
    nh_.param("marker_lifetime", lifetime_, DEFAULT_LIFETIME);
    nh_.param("z_pos", z_pos_, DEFAULT_Z_POSITION);
    nh_.param("marker_disabled", disabled_, false);

    // Create the publisher
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>(topic_name_, 1);

    vx_last_ = 0.0;
    vy_last_ = 0.0;
    vtheta_last_ = 0.0;

    // Create the markers
    createDirectionalMarkers();
    createRotationalMarkers();
}


VelocityLimitedMarker::~VelocityLimitedMarker()
{
}


void VelocityLimitedMarker::createDirectionalMarkers()
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
    marker.pose.position.z = z_pos_;
    marker.scale.x = MARKER_SCALE_DIR;
    marker.scale.y = MARKER_SCALE_DIR;
    marker.scale.z = 1.0;
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
        float cosa = cos(a);
        float sina = sin(a);

        v1.x = MARKER_RADIUS_DIR * cosa;
        v1.y = MARKER_RADIUS_DIR * sina;
        v2.x = (MARKER_RADIUS_DIR + MARKER_WIDTH_DIR) * cosa;
        v2.y = (MARKER_RADIUS_DIR + MARKER_WIDTH_DIR) * sina;

        circle1.push_back(v1);
        circle2.push_back(v2);
    }

    marker.points.clear();
    for( std::size_t i = 0; i < (circle1.size() - 1); ++i )
    {
        std::size_t i1 = i;
        std::size_t i2 = (i + 1) % circle1.size();

        marker.points.push_back(circle1[i1]);
        marker.points.push_back(circle2[i1]);
        marker.points.push_back(circle1[i2]);

        marker.points.push_back(circle1[i2]);
        marker.points.push_back(circle2[i2]);
        marker.points.push_back(circle2[i1]);
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


void VelocityLimitedMarker::createRotationalMarkers()
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
    marker.pose.position.z = z_pos_;
    marker.scale.x = MARKER_SCALE_ROT;
    marker.scale.y = MARKER_SCALE_ROT;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5; // adjust according to the velocity?

    // Create the disc like geometry for the markers
    std::vector<geometry_msgs::Point> circle1, circle2, circle3;
    geometry_msgs::Point v1, v2, v3;

    static const int STEPS = 48;
    static const int FIRST = 0;
    static const int LAST = 23;
    for( int i = FIRST; i <= LAST; ++i )
    {
        float a = float(2*i) / float(STEPS) * M_PI * 2.0;
        float cosa = cos(a);
        float sina = sin(a);

        v1.x = MARKER_RADIUS_ROT * cosa;
        v1.y = MARKER_RADIUS_ROT * sina;
        v2.x = (MARKER_RADIUS_ROT + MARKER_WIDTH_ROT) * cosa;
        v2.y = (MARKER_RADIUS_ROT + MARKER_WIDTH_ROT) * sina;

        circle1.push_back(v1);
        circle2.push_back(v2);

        a = float(2*i+1) / float(STEPS) * M_PI * 2.0;
        cosa = cos(a);
        sina = sin(a);

        v3.x = (MARKER_RADIUS_ROT + 0.5 * MARKER_WIDTH_ROT) * cosa;
        v3.y = (MARKER_RADIUS_ROT + 0.5 * MARKER_WIDTH_ROT) * sina;

        circle3.push_back(v3);
    }

    marker.points.clear();
    for( std::size_t i = 0; i < circle1.size(); ++i )
    {
        marker.points.push_back(circle1[i]);
        marker.points.push_back(circle2[i]);
        marker.points.push_back(circle3[i]);
    }

    // Particular messages for each axis
    theta_pos_marker_ = marker;
    theta_pos_marker_.id = 4;
    theta_pos_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    theta_neg_marker_ = marker;
    theta_neg_marker_.id = 5;
    theta_neg_marker_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0, 0);
}


void VelocityLimitedMarker::interpolateColor(double velocity, std_msgs::ColorRGBA& color)
{
	if( velocity < MIN_VELOCITY )
	{
		color.r = color.g = color.b = color.a = 0.0f;
		return;
	}

    float alpha = float((velocity - MIN_VELOCITY) * VELOCITY_COEFF);
    alpha = (alpha > 1.0f) ? 1.0f : alpha;

    color.r = (1.0f - alpha) * MIN_COLOR[0] + alpha * MAX_COLOR[0];
    color.g = (1.0f - alpha) * MIN_COLOR[1] + alpha * MAX_COLOR[1];
    color.b = (1.0f - alpha) * MIN_COLOR[2] + alpha * MAX_COLOR[2];
    color.a = (1.0f - alpha) * MIN_COLOR[3] + alpha * MAX_COLOR[3];
}


void VelocityLimitedMarker::publishMarkers( double vel_x_desired, 
                                            double vel_x_actual, 
                                            double vel_y_desired, 
                                            double vel_y_actual, 
                                            double vel_theta_desired,
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
    double x_vel_diff = fabs(vel_x_desired - vel_x_actual);
    if( x_vel_diff >= epsilon )
    {
//        double alpha = x_vel_diff * VELOCITY_COEFF;
        if (vel_x_desired >= 0.0 && ax <= 0.0)
        {
            x_pos_marker_.header.stamp = ros::Time::now();
            interpolateColor(x_vel_diff, x_pos_marker_.color);
//            x_pos_marker_.color.a = (alpha > 1.0) ? 1.0 : alpha;
        	marker_pub_.publish(x_pos_marker_);            
        }
        else if (vel_x_desired <= 0.0 && ax >= 0.0)
        {
            x_neg_marker_.header.stamp = ros::Time::now();
            interpolateColor(x_vel_diff, x_neg_marker_.color);
//            x_neg_marker_.color.a = (alpha > 1.0) ? 1.0 : alpha;
    	    marker_pub_.publish(x_neg_marker_);
        }
    }

    // for y-axis
    double y_vel_diff = fabs(vel_y_desired - vel_y_actual);
    if( y_vel_diff >= epsilon )
    {
//        double alpha = y_vel_diff * VELOCITY_COEFF;
        if (vel_y_desired >= 0.0 && ay <= 0.0)
        {
            y_pos_marker_.header.stamp = ros::Time::now();
            interpolateColor(y_vel_diff, y_pos_marker_.color);
//            y_pos_marker_.color.a = (alpha > 1.0) ? 1.0 : alpha;
        	marker_pub_.publish(y_pos_marker_);            
        }
        else if (vel_y_desired <= 0.0 && ay >= 0.0)
        {
            y_neg_marker_.header.stamp = ros::Time::now();
            interpolateColor(y_vel_diff, y_neg_marker_.color);
//            y_neg_marker_.color.a = (alpha > 1.0) ? 1.0 : alpha;
    	    marker_pub_.publish(y_neg_marker_);
        }
    }

    // for theta-axis
    double theta_vel_diff = fabs(vel_theta_desired - vel_theta_actual);
    if( theta_vel_diff >= epsilon )
    {
//        double alpha = theta_vel_diff * VELOCITY_COEFF;
        if (vel_theta_desired >= 0.0  && atheta <= 0.0)
        {
            theta_pos_marker_.header.stamp = ros::Time::now();
            interpolateColor(theta_vel_diff, theta_pos_marker_.color);
//            theta_pos_marker_.color.a = (alpha > 1.0) ? 1.0 : alpha;
        	marker_pub_.publish(theta_pos_marker_);            
        }
        else if (vel_theta_desired <= 0.0  && atheta >= 0.0)
        {
            theta_neg_marker_.header.stamp = ros::Time::now();
            interpolateColor(theta_vel_diff, theta_neg_marker_.color);
//            theta_neg_marker_.color.a = (alpha > 1.0) ? 1.0 : alpha;
    	    marker_pub_.publish(theta_neg_marker_);
        }
    }
}


}

