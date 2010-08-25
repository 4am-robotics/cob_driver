/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_point_cloud_publisher
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: August 2010
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

//##################
//#### includes ####

// standard includes
#include <cob_point_cloud_publisher/cob_colored_point_cloud_publisher.h>


using namespace message_filters;


        // Constructor
PcPublisher::PcPublisher()
	: image_transport_(n_),
	pc_sync_(3),
	c_xyz_image_32F3_(0),
	c_confidence_mask_32F1_(0),
	c_color_image_8U3_(0)
{
}

void PcPublisher::onInit()
{
	n_ = getNodeHandle();
	image_transport_ = image_transport::ImageTransport(n_);
	initNode();
}
        
void PcPublisher::initNode()
{
	topicPub_pointCloud_ = n_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
	xyz_image_subscriber_.subscribe(image_transport_, "image_xyz", 1);
	confidence_mask_subscriber_.subscribe(image_transport_,"image_confidence", 1);
	color_image_subscriber_.subscribe(image_transport_,"image_color", 1);
	feature_mask_subscriber_.subscribe(image_transport_,"image_features", 1);

	pc_sync_.connectInput(xyz_image_subscriber_, confidence_mask_subscriber_, color_image_subscriber_,feature_mask_subscriber_);
	pc_sync_.registerCallback(boost::bind(&PcPublisher::syncCallback, this, _1, _2, _3, _4));
}


// topic callback functions
// function will be called when a new message arrives on a topic
void PcPublisher::syncCallback(const sensor_msgs::Image::ConstPtr& pc_xyz_data,
		const sensor_msgs::Image::ConstPtr& pc_confidence_data,
		const sensor_msgs::Image::ConstPtr& pc_color_data,
		const sensor_msgs::Image::ConstPtr& pc_feature_data)
{
	ROS_DEBUG("convert xyz_image to point_cloud");
	sensor_msgs::PointCloud2 pc_msg;
	// create point_cloud message
	pc_msg.header.stamp = ros::Time::now();
	pc_msg.header.frame_id = "head_tof_camera_link";
	c_xyz_image_32F3_ = cv_bridge_0_.imgMsgToCv(pc_xyz_data, "passthrough");
	c_confidence_mask_32F1_ = cv_bridge_1_.imgMsgToCv(pc_confidence_data, "passthrough");
	c_color_image_8U3_ = cv_bridge_2_.imgMsgToCv(pc_color_data, "passthrough");
	c_feature_mask_8U1_ = cv_bridge_3_.imgMsgToCv(pc_feature_data, "passthrough");
	cv::Mat cpp_xyz_image_32F3 = c_xyz_image_32F3_;
	cv::Mat cpp_confidence_mask_32F1 = c_confidence_mask_32F1_;
	cv::Mat cpp_color_image_8U3 = c_color_image_8U3_;
	cv::Mat cpp_feature_mask_8U1 = c_feature_mask_8U1_;

	pc_msg.width = cpp_xyz_image_32F3.cols;
	pc_msg.height = cpp_xyz_image_32F3.rows;
	pc_msg.fields.resize(6);
	pc_msg.fields[0].name = "x";
	pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pc_msg.fields[1].name = "y";
	pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	pc_msg.fields[2].name = "z";
	pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	pc_msg.fields[3].name = "bgr";
	pc_msg.fields[3].datatype = sensor_msgs::PointField::UINT32;
	pc_msg.fields[4].name = "confidence";
	pc_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
	pc_msg.fields[5].name = "features";
	pc_msg.fields[5].datatype = sensor_msgs::PointField::UINT8;
	int offset = 0;
	for (size_t d = 0; d < pc_msg.fields.size(); ++d, offset += 4)
	{
		pc_msg.fields[d].offset = offset;
	}
	pc_msg.point_step = offset-3;
	pc_msg.row_step = pc_msg.point_step * pc_msg.width;
	pc_msg.data.resize (pc_msg.width*pc_msg.height*pc_msg.point_step);
	pc_msg.is_dense = true;
	pc_msg.is_bigendian = false;

	float* f_ptr = 0;
	unsigned char* c_ptr = 0;
	float* g_ptr = 0;
	unsigned char* ft_ptr = 0;
	int pc_msg_idx=0;
	for (int row = 0; row < cpp_xyz_image_32F3.rows; row++)
	{
		f_ptr = cpp_xyz_image_32F3.ptr<float>(row);
		c_ptr = cpp_color_image_8U3.ptr<unsigned char>(row);
		g_ptr = cpp_confidence_mask_32F1.ptr<float>(row);
		ft_ptr = cpp_feature_mask_8U1.ptr<unsigned char>(row);
		for (int col = 0; col < cpp_xyz_image_32F3.cols; col++, pc_msg_idx++)
		{
			memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step], &f_ptr[3*col], 3*sizeof(float));
			memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step + pc_msg.fields[3].offset], &c_ptr[3*col], 3*sizeof(unsigned char));
			memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step + pc_msg.fields[4].offset], &g_ptr[col], sizeof(float));
			memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step + pc_msg.fields[5].offset], &ft_ptr[col], sizeof(unsigned char));
		}
	}

	pc_msg.header.stamp = ros::Time::now();
	topicPub_pointCloud_.publish(pc_msg);
}



