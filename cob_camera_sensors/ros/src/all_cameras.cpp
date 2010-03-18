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
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>

// external includes
#include <cob_camera_sensors/AbstractColorCamera.h>
#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <cob_vision_utils/GlobalDefines.h>

using namespace ipa_CameraSensors;

/// @class CobColorCameraNode
/// ROS node to interface color cameras.
class CobAllCamerasNode
{
private:
	ros::NodeHandle node_handle_;

	AbstractColorCamera* left_color_camera_;	///< Color camera instance
	AbstractColorCamera* right_color_camera_;	///< Color camera instance
	AbstractRangeImagingSensor* tof_camera_;	///< Time-of-flight camera instance

	sensor_msgs::CameraInfo left_color_camera_info_message_;	///< ROS camera information message (e.g. holding intrinsic parameters)
	sensor_msgs::CameraInfo right_color_camera_info_message_;	///< ROS camera information message (e.g. holding intrinsic parameters)
	sensor_msgs::CameraInfo tof_camera_info_message_;	///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer left_color_camera_info_service_;
	ros::ServiceServer right_color_camera_info_service_;
	ros::ServiceServer tof_camera_info_service_;

	IplImage* left_color_image_8U3_;	///< color image of left camera
	IplImage* right_color_image_8U3_;	///< color image of right camera
	IplImage* xyz_tof_image_32F3_;      /// OpenCV image holding the point cloud from tof sensor
	IplImage* grey_tof_image_32F1_;     /// OpenCV image holding the amplitude values of the point cloud

	image_transport::ImageTransport image_transport_;	///< Image transport instance
	image_transport::Publisher xyz_tof_image_publisher_;	///< Publishes xyz image data
	image_transport::Publisher grey_tof_image_publisher_;	///< Publishes grey image data
	image_transport::Publisher left_color_image_publisher_;	///< Publishes grey image data
	image_transport::Publisher right_color_image_publisher_;	///< Publishes grey image data

public:
	CobAllCamerasNode(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  left_color_camera_(0),
	  right_color_camera_(0),
	  tof_camera_(0),
	  left_color_image_8U3_(0),
	  right_color_image_8U3_(0),
	  xyz_tof_image_32F3_(0),
	  grey_tof_image_32F1_(0),
	  image_transport_(node_handle)
	{
		/// Void
	}

	~CobAllCamerasNode()
	{
		if (left_color_camera_)
		{
			left_color_camera_->Close();
			ipa_CameraSensors::ReleaseColorCamera(left_color_camera_);
		}
		if (right_color_camera_)
		{
			right_color_camera_->Close();
			ipa_CameraSensors::ReleaseColorCamera(right_color_camera_);
		}
		if (tof_camera_)
		{
			tof_camera_->Close();
			ipa_CameraSensors::ReleaseRangeImagingSensor(tof_camera_);
		}
		
		if (left_color_image_8U3_) cvReleaseImage(&left_color_image_8U3_);
		if (right_color_image_8U3_) cvReleaseImage(&right_color_image_8U3_);
		if (xyz_tof_image_32F3_) cvReleaseImage(&xyz_tof_image_32F3_);
		if (grey_tof_image_32F1_) cvReleaseImage(&grey_tof_image_32F1_);
	} 

	/// Opens the camera sensor
	bool init()
	{
		std::string directory = "NULL/";

		/// Parameters are set within the launch file
		if (node_handle_.getParam("color_camera/configuration_files", directory) == false)
		{
			ROS_ERROR("Path to xml configuration for color camera not specified");
			return false;
		}

		// Initialize left camera if possible
		left_color_camera_ = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();
	
		if (left_color_camera_->Init(directory, 0) & ipa_CameraSensors::RET_FAILED)
		{
			ROS_WARN("[all_cameras] Initialization of left camera (0) failed");
			left_color_camera_ = 0;
		}

		if (left_color_camera_ && (left_color_camera_->Open() & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Opening left color camera (0) failed");
			left_color_camera_ = 0;
		}

		// Initialize right camera if possible
		right_color_camera_ = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();
	
		if (right_color_camera_->Init(directory, 0) & ipa_CameraSensors::RET_FAILED)
		{
			ROS_WARN("[all_cameras] Initialization of right camera (1) failed");
			right_color_camera_ = 0;
		}

		if (right_color_camera_ && (right_color_camera_->Open() & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Opening right color camera (1) failed");
			right_color_camera_ = 0;
		}

		// Initialize right camera if possible
		tof_camera_ = ipa_CameraSensors::CreateRangeImagingSensor_Swissranger();
		
		if (tof_camera_->Init(directory) & ipa_CameraSensors::RET_FAILED)
		{
			ROS_WARN("[all_cameras] Initialization of tof camera (0) failed");
			tof_camera_ = 0;
		}

		if (tof_camera_->Open() & ipa_CameraSensors::RET_FAILED)
		{
			ROS_WARN("[all_cameras] Opening tof camera (0) failed");
			tof_camera_ = 0;
		}

		/// Advertise service for other nodes to set intrinsic calibration parameters
		left_color_camera_info_service_ = node_handle_.advertiseService("set_camera_info", &CobAllCamerasNode::setCameraInfo, this);
		right_color_camera_info_service_ = node_handle_.advertiseService("set_camera_info", &CobAllCamerasNode::setCameraInfo, this);
		tof_camera_info_service_ = node_handle_.advertiseService("set_camera_info", &CobAllCamerasNode::setCameraInfo, this);
	
		/// Topics to publish
		if (left_color_camera_) left_color_image_publisher_ = image_transport_.advertise("left/color_camera_stream/image_raw", 1);
		if (right_color_camera_) right_color_image_publisher_ = image_transport_.advertise("right/color_camera_stream/image_raw", 1);
		if (tof_camera_)
		{
			 grey_tof_image_publisher_ = image_transport_.advertise("grey_tof_data", 1);
			 xyz_tof_image_publisher_ = image_transport_.advertise("xyz_tof_data", 1);
		}
	
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
		//camera_info_message_ = req.camera_info;
    
		rsp.success = false;
	        rsp.status_message = "[all_cameras] Setting camera parameters through ROS not implemented";

    		return true;
  	}

	/// Callback function for image requests on topic 'request_image'
	void spin()
	{ 
		ros::Rate rate(3);
		while(node_handle_.ok())
		{

		// ROS images
		sensor_msgs::Image right_color_image_msg;
		sensor_msgs::Image left_color_image_msg;
		sensor_msgs::Image xyz_tof_image_msg;
		sensor_msgs::Image grey_tof_image_msg;
	
		// ROS camera information messages
		sensor_msgs::CameraInfo right_color_image_info;
		sensor_msgs::CameraInfo left_color_image_info;
		sensor_msgs::CameraInfo tof_image_info;

		ros::Time now = ros::Time::now();

		// Acquire left color image
		if (left_color_camera_)
		{
	   		/// Release previously acquired IplImage 
			if (left_color_image_8U3_) 
			{
				cvReleaseImage(&left_color_image_8U3_);
				left_color_image_8U3_ = 0;
			}
	
			/// Acquire new image
			if (left_color_camera_->GetColorImage2(&left_color_image_8U3_) & ipa_Utils::RET_FAILED)
			{
				ROS_ERROR("[all_cameras] Left color image acquisition failed");
				left_color_camera_ = 0;
				break;
			}

			try
	  		{
				left_color_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(left_color_image_8U3_, "bgr8"));
			}
			catch (sensor_msgs::CvBridgeException error)
			{
				ROS_ERROR("[all_cameras] Could not convert left IplImage to ROS message");
				break;
			}
			left_color_image_msg.header.stamp = now;    
	
			left_color_image_info = left_color_camera_info_message_;
			left_color_image_info.width = left_color_image_8U3_->width;
			left_color_image_info.height = left_color_image_8U3_->height;
			left_color_image_info.header.stamp = now;

			left_color_image_publisher_.publish(left_color_image_msg);
		}
	
		// Acquire right color image
		if (right_color_camera_)
		{
	   		/// Release previously acquired IplImage 
			if (right_color_image_8U3_) 
			{
				cvReleaseImage(&right_color_image_8U3_);
				right_color_image_8U3_ = 0;
			}
	
			/// Acquire new image
			if (right_color_camera_->GetColorImage2(&right_color_image_8U3_) & ipa_Utils::RET_FAILED)
			{
				ROS_ERROR("[all_cameras] Right color image acquisition failed");
				right_color_camera_ = 0;
				break;
			}

			try
	  		{
				right_color_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(right_color_image_8U3_, "bgr8"));
			}
			catch (sensor_msgs::CvBridgeException error)
			{
				ROS_ERROR("[all_cameras] Could not convert right IplImage to ROS message");
				break;
			}
			right_color_image_msg.header.stamp = now;    
	
			right_color_image_info = right_color_camera_info_message_;
			right_color_image_info.width = right_color_image_8U3_->width;
			right_color_image_info.height = right_color_image_8U3_->height;
			right_color_image_info.header.stamp = now;

			right_color_image_publisher_.publish(right_color_image_msg);
		}
		
		// Acquire image from tof camera	
		if (tof_camera_)
		{
	                /// Release previously acquired IplImage 
	                if (xyz_tof_image_32F3_)
	                {
	                        cvReleaseImage(&xyz_tof_image_32F3_);
	                        xyz_tof_image_32F3_ = 0;
	                }
	
	                if (grey_tof_image_32F1_)
	                {
	                        cvReleaseImage(&grey_tof_image_32F1_);
	                        grey_tof_image_32F1_ = 0;
	                }
	
			if(tof_camera_->AcquireImages2(0, &grey_tof_image_32F1_, &xyz_tof_image_32F3_, false, false, ipa_CameraSensors::AMPLITUDE) & ipa_Utils::RET_FAILED)
			{
				ROS_ERROR("[all_cameras] Tof image acquisition failed");
	                        tof_camera_ = 0;
				break;	
			}

			try
	                {
	                        xyz_tof_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(xyz_tof_image_32F3_, "passthrough"));
	                        grey_tof_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(grey_tof_image_32F1_, "passthrough"));
	                }
	                catch (sensor_msgs::CvBridgeException error)
	                {
	                        ROS_ERROR("[all_cameras] Could not convert tof IplImage to ROS message");
				break;
	                }

			xyz_tof_image_msg.header.stamp = now;    
			grey_tof_image_msg.header.stamp = now;    
	
			tof_image_info = tof_camera_info_message_;
			tof_image_info.width = left_color_image_8U3_->width;
			tof_image_info.height = left_color_image_8U3_->height;
			tof_image_info.header.stamp = now;
			
			grey_tof_image_publisher_.publish(grey_tof_image_msg);
			xyz_tof_image_publisher_.publish(xyz_tof_image_msg);
		}
		
		ros::spinOnce();
		rate.sleep();

		} // END while-loop
		
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
	CobAllCamerasNode camera_node(nh);

	/// initialize camera node
	if (!camera_node.init()) return 0;

	camera_node.spin();
	
	return 0;
}
