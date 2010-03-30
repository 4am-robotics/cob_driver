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
 * ROS stack name: cob3_driver
 * ROS package name: cob3_camera_sensors
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Mar 2010
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

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>


class CobCameraSynchronizerNode
{
private:
	ros::NodeHandle node_handle_;


	sensor_msgs::CameraInfo camera_info_message_;	///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer camera_info_service_;

	IplImage* image_;	///< image of camera


	image_transport::ImageTransport image_transport_;	///< Image transport instance
	image_transport::CameraPublisher image_publisher_;	///< Publishes image data

	image_transport::CameraSubscriber image_subscriber_;	///< Subscribes to image data


public:
	CobCameraSynchronizerNode(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  image_(0),
	  image_transport_(node_handle)
	{
		/// Void
	}

	~CobCameraSynchronizerNode()
	{
		if (image_) cvReleaseImage(&image_);
	}

	/// Opens the camera sensor
	bool init()
	{
		/// Topics and Services to publish
		image_publisher_ = image_transport_.advertiseCamera("test/image_sync", 1);
		image_subscriber_ = image_transport_.subscribeCamera("image_non_sync", 1, &CobCameraSynchronizerNode::topicCallback_image, this);
		return true;
	}


	/// Callback function for image requests on topic 'request_image'
	void topicCallback_image(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& image_info)
	{
		// ROS images
		sensor_msgs::Image out_image_msg = *image_msg;

		// ROS camera information messages
		sensor_msgs::CameraInfo out_image_info = *image_info;

		ros::Time now = ros::Time::now();

		out_image_msg.header.stamp = now;
		out_image_info.header.stamp = now;


		image_publisher_.publish(out_image_msg, out_image_info);
	}

};



//#######################
//#### main program ####
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "camera_synchronizer");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/*std::string node_name = ros::this_node::getName().c_str();
	//ROS_INFO("Name: %s", ros::this_node::getName().c_str());

	std::string sub_topic_name;

	if (nh.getParam(node_name+"/sub_topic_name", sub_topic_name) == false)
	{
		ROS_ERROR("[camera_synchronizer] Topic to subscribe to not specified.");
		return false;
	}

	std::string pub_topic_name;

	if (nh.getParam(node_name+"/pub_topic_name", pub_topic_name) == false)
	{
		ROS_ERROR("[camera_synchronizer] Topic to publish not specified.");
		return false;
	}*/

	/// Create camera node class instance
	CobCameraSynchronizerNode camera_sync_node(nh);

	/// initialize camera node
	if (camera_sync_node.init() == false)
	{
		ROS_ERROR("[all_cameras] Node initialization FAILED. Terminating");
		return 0;
	}
	else
	{
		ROS_INFO("[all_cameras] Node initialization OK. Enter spinning");
	}

	ros::Rate rate(3);
	while(nh.ok())
	{
		ros::spinOnce();
		rate.sleep();
	} // END while-loop

	return 0;
}
