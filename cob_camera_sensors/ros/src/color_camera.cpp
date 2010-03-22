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
 * Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
 * Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
 *
 * Date of creation: Jan 2010
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
//--

// ROS includes
#include <ros/ros.h>
#include <polled_camera/publication_server.h>
#include <cv_bridge/CvBridge.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>

// external includes
#include <cob_camera_sensors/AbstractColorCamera.h>
#include <cob_vision_utils/GlobalDefines.h>

using namespace ipa_CameraSensors;

/// @class CobColorCameraNode
/// ROS node to interface color cameras.
class CobColorCameraNode
{
private:
	ros::NodeHandle node_handle_;
	polled_camera::PublicationServer image_poll_server_;

	AbstractColorCamera* color_camera_;	///< Color camera instance

	sensor_msgs::CameraInfo camera_info_message_;	///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer camera_info_service_;

	IplImage* color_image_8U3_;

public:
	CobColorCameraNode(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  color_camera_(0),
	  color_image_8U3_(0)
	{
		/// Void
	}

	~CobColorCameraNode()
	{
		image_poll_server_.shutdown();
		color_camera_->Close();
		ipa_CameraSensors::ReleaseColorCamera(color_camera_);
		
		if (color_image_8U3_) cvReleaseImage(&color_image_8U3_);
	} 

	/// Opens the camera sensor
	bool init()
	{
		std::string tmp_string = "NULL";
		int camera_index = -1;
		std::string directory = "NULL/";

		/// Parameters are set within the launch file
		if (node_handle_.getParam("color_camera/camera_index", camera_index) == false)
		{
			ROS_ERROR("[color_camera] Color camera index (0 or 1) not specified");
			return false;
		}
	
		/// Parameters are set within the launch file
		if (node_handle_.getParam("color_camera/configuration_files", directory) == false)
		{
			ROS_ERROR("[color_camera] Path to xml configuration for color camera not specified");
			return false;
		}

		/// Parameters are set within the launch file
		if (node_handle_.getParam("color_camera/color_camera_type", tmp_string) == false)
		{
			ROS_ERROR("[color_camera] Color camera type not specified");
			return false;
		}
		if (tmp_string == "CAM_AVTPIKE") color_camera_ = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();
		else if (tmp_string == "CAM_PROSILICA") ROS_ERROR("[color_camera] Color camera type not CAM_PROSILICA not yet implemented");
		else
		{
			std::string str = "[color_camera] Camera type '" + tmp_string + "' unknown, try 'CAM_AVTPIKE' or 'CAM_PROSILICA'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}
	
		if (color_camera_->Init(directory, camera_index) & ipa_CameraSensors::RET_FAILED)
		{
			std::stringstream ss;
			ss << "initialization of color camera ";
			ss << camera_index;
			ss << " failed"; 
			ROS_ERROR("[color_camera] %s", ss.str().c_str());
			color_camera_ = 0;
			return false;
		}

		if (color_camera_ && (color_camera_->Open() & ipa_CameraSensors::RET_FAILED))
		{
			std::stringstream ss;
			ss << "Could not open color camera ";
			ss << camera_index;
			ROS_ERROR("[color_camera] %s", ss.str().c_str());
			color_camera_ = 0;
			return false;
		}

		/// Advertise service for other nodes to set intrinsic calibration parameters
		camera_info_service_ = node_handle_.advertiseService("set_camera_info", &CobColorCameraNode::setCameraInfo, this);
	
		/// Topics to publish
		image_poll_server_ = polled_camera::advertise(node_handle_, "request_image", &CobColorCameraNode::pollCallback, this);
		
		return true;
	}

	/// Enables the user to modify camera parameters.
	/// @param req Requested camera parameters
	/// @param rsp Response, telling if requested parameters have been set
	/// @return <code>True</code>
	bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
			sensor_msgs::SetCameraInfo::Response& rsp)
	{
		/// @TODO: Enable the setting of intrinsic parameters
		camera_info_message_ = req.camera_info;
    
		rsp.success = false;
	        rsp.status_message = "[color_camera] Setting camera parameters through ROS not implemented";

    		return true;
  	}

	/// Callback function for image requests on topic 'request_image'
	bool pollCallback(polled_camera::GetPolledImage::Request& req, 
			sensor_msgs::Image& image_msg, sensor_msgs::CameraInfo& info)
	{
   		/// Release previously acquired IplImage 
		if (color_image_8U3_) 
		{
			cvReleaseImage(&color_image_8U3_);
			color_image_8U3_ = 0;
		}

		/// Acquire new image
		if (color_camera_->GetColorImage2(&color_image_8U3_) & ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("[color_camera] Color image acquisition failed");
			return false;
		}

		try
  		{
			image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(color_image_8U3_, "bgr8"));
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("[color_camera] Could not convert IplImage to ROS message");
		}
	
		/// Set time stamp
		ros::Time now = ros::Time::now();
		image_msg.header.stamp = now;    

		info = camera_info_message_;
		info.width = color_image_8U3_->width;
		info.height = color_image_8U3_->height;
		info.header.stamp = now;

    		return true;
	}
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, spezify name of node
	ros::init(argc, argv, "color_camera");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;
	
	/// Create camera node class instance	
	CobColorCameraNode camera_node(nh);

	/// initialize camera node
	if (!camera_node.init()) return 0;

	ros::spin();
	
	return 0;
}
