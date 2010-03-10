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

class CobColorCameraNode
{
private:
	ros::NodeHandle m_NodeHandle;
	polled_camera::PublicationServer m_ImagePollServer;

	AbstractColorCamera* m_ColorCamera;	///< Color camera instance

	sensor_msgs::Image m_ImageMessage;	///< ROS image message
	sensor_msgs::CameraInfo m_CameraInfoMessage;	///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer m_CameraInfoService;

	IplImage* m_IplImage;

public:
	CobColorCameraNode(const ros::NodeHandle& node_handle)
	: m_NodeHandle(node_handle),
	  m_ColorCamera(0),
	  m_IplImage(0)
	{
		/// Void
	}

	~CobColorCameraNode()
	{
		m_ImagePollServer.shutdown();
		m_ColorCamera->Close();
		ipa_CameraSensors::ReleaseColorCamera(m_ColorCamera);
		
		if (m_IplImage) cvReleaseImage(&m_IplImage);
	} 

	/// Opens the camera sensor
	bool Init()
	{
		int cameraIndex = -1;
		std::string directory = "NULL/";

		/// Parameters are set within the launch file
		if (m_NodeHandle.getParam("color_camera/camera_index", cameraIndex) == false)
		{
			ROS_ERROR("Color camera index (0 or 1) not specified");
			return false;
		}
	
		/// Parameters are set within the launch file
		if (m_NodeHandle.getParam("color_camera/configuration_files", directory) == false)
		{
			ROS_ERROR("Path to xml configuration for color camera not specified");
			return false;
		}

		m_ColorCamera = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();
	
		if (m_ColorCamera->Init(directory, cameraIndex) & ipa_CameraSensors::RET_FAILED)
		{
			std::stringstream ss;
			ss << "Initialization of color camera ";
			ss << cameraIndex;
			ss << " failed"; 
			ROS_ERROR("%s", ss.str().c_str());
			m_ColorCamera = 0;
			return false;
		}

		if (m_ColorCamera && (m_ColorCamera->Open() & ipa_CameraSensors::RET_FAILED))
		{
			std::stringstream ss;
			ss << "Could not open color camera ";
			ss << cameraIndex;
			ROS_ERROR("%s", ss.str().c_str());
			m_ColorCamera = 0;
			return false;
		}

		/// Advertise service for other nodes to set intrinsic calibration parameters
		m_CameraInfoService = m_NodeHandle.advertiseService("set_camera_info", &CobColorCameraNode::SetCameraInfo, this);
	
		/// Topics to publish
		m_ImagePollServer = polled_camera::advertise(m_NodeHandle, "request_image", &CobColorCameraNode::PollCallback, this);
		
		return true;
	}

	/// Enables the user to modify camera parameters.
	/// @param req Requested camera parameters
	/// @param rsp Response, telling if requested parameters have been set
	/// @return <code>True</code>
	bool SetCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
			sensor_msgs::SetCameraInfo::Response& rsp)
	{
		/// @TODO: Enable the setting of intrinsic parameters
		m_CameraInfoMessage = req.camera_info;
    
		rsp.success = false;
	        rsp.status_message = "Setting camera parameters through ROS not implemented";

    		return true;
  	}

	/// Callback function for image requests on topic 'request_image'
	bool PollCallback(polled_camera::GetPolledImage::Request& req, 
			sensor_msgs::Image& imageMsg, sensor_msgs::CameraInfo& info)
	{
   		/// Release previously acquired IplImage 
		if (m_IplImage) 
		{
			cvReleaseImage(&m_IplImage);
			m_IplImage = 0;
		}

		/// Acquire new image
		if (m_ColorCamera->GetColorImage2(&m_IplImage) & ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("Color image acquisition failed");
			return false;
		}

		try
  		{
			imageMsg = *(sensor_msgs::CvBridge::cvToImgMsg(m_IplImage, "bgr8"));
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("Could not convert IplImage to ROS message");
		}
	
		/// Set time stamp
		imageMsg.header.stamp = ros::Time::now();    

		info = m_CameraInfoMessage;
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
	CobColorCameraNode cameraNode(nh);

	/// Initialize camera node
	if (!cameraNode.Init()) return 0;

	ros::spin();
	
	return 0;
}
