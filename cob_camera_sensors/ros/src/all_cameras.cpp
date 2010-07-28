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
 * ROS package name: cob_camera_sensors
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
#include <cob_vision_utils/CameraSensorToolbox.h>

using namespace ipa_CameraSensors;

/// @class CobColorCameraNode
/// ROS node to interface color cameras.
class CobAllCamerasNode
{
private:
	ros::NodeHandle node_handle_;

	AbstractColorCameraPtr left_color_camera_;	///< Color camera instance
	AbstractColorCameraPtr right_color_camera_;	///< Color camera instance
	AbstractRangeImagingSensorPtr tof_camera_;	///< Time-of-flight camera instance
	
	std::string config_directory_;	///< Directory of the configuration files
		
	std::string left_color_camera_ns_; ///< Namespace name of left color camera
	std::string right_color_camera_ns_; ///< Namespace name of left color camera
	std::string tof_camera_ns_; ///< Namespace name of left color camera

	int left_color_camera_intrinsic_id_;	///< Instrinsic matrix id of left color camera
	int right_color_camera_intrinsic_id_;	///< Instrinsic matrix id of right color camera
	int tof_camera_intrinsic_id_;	///< Instrinsic matrix id of tof camera

	ipa_CameraSensors::t_cameraType left_color_camera_intrinsic_type_;	///< Instrinsic matrix type of left color camera
	ipa_CameraSensors::t_cameraType right_color_camera_intrinsic_type_;	///< Instrinsic matrix type of right color camera
	ipa_CameraSensors::t_cameraType tof_camera_intrinsic_type_;	///< Instrinsic matrix type of tof camera

	sensor_msgs::CameraInfo left_color_camera_info_msg_;	///< ROS camera information message (e.g. holding intrinsic parameters)
	sensor_msgs::CameraInfo right_color_camera_info_msg_;	///< ROS camera information message (e.g. holding intrinsic parameters)
	sensor_msgs::CameraInfo tof_camera_info_msg_;	///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer left_color_camera_info_service_;
	ros::ServiceServer right_color_camera_info_service_;
	ros::ServiceServer tof_camera_info_service_;

	cv::Mat left_color_image_8U3_;	///< color image of left camera
	cv::Mat right_color_image_8U3_;	///< color image of right camera
	cv::Mat xyz_tof_image_32F3_;      /// OpenCV image holding the point cloud from tof sensor
	cv::Mat grey_tof_image_32F1_;     /// OpenCV image holding the amplitude values of the point cloud

	image_transport::ImageTransport image_transport_;	///< Image transport instance
	image_transport::CameraPublisher xyz_tof_image_publisher_;	///< Publishes xyz image data
	image_transport::CameraPublisher grey_tof_image_publisher_;	///< Publishes grey image data
	image_transport::CameraPublisher left_color_image_publisher_;	///< Publishes grey image data
	image_transport::CameraPublisher right_color_image_publisher_;	///< Publishes grey image data

public:
	CobAllCamerasNode(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  left_color_camera_(AbstractColorCameraPtr()),
	  right_color_camera_(AbstractColorCameraPtr()),
	  tof_camera_(AbstractRangeImagingSensorPtr()),
	  left_color_image_8U3_(cv::Mat()),
	  right_color_image_8U3_(cv::Mat()),
	  xyz_tof_image_32F3_(cv::Mat()),
	  grey_tof_image_32F1_(cv::Mat()),
	  image_transport_(node_handle)
	{
		/// Void
	}

	~CobAllCamerasNode()
	{
		ROS_INFO("[all_cameras] Shutting down cameras");
		if (left_color_camera_)
		{
			ROS_INFO("[all_cameras] Shutting down left color camera (1)");
			left_color_camera_->Close();
		}
		if (right_color_camera_)
		{
			ROS_INFO("[all_cameras] Shutting down right color camera (0)");
			right_color_camera_->Close();
		}
		if (tof_camera_)
		{
			ROS_INFO("[all_cameras] Shutting down tof camera (0)");
			tof_camera_->Close();
		}
	} 

	/// Opens the camera sensor
	bool init()
	{
		if (loadParameters() == false)
		{
			ROS_ERROR("[all_cameras] Could not read parameters from launch file");
			return false;
		}
		
		if (left_color_camera_ && (left_color_camera_->Init(config_directory_, 1) & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Initialization of left camera (1) failed");
			left_color_camera_ = AbstractColorCameraPtr();
		}

		if (left_color_camera_ && (left_color_camera_->Open() & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Opening left color camera (1) failed");
			left_color_camera_ = AbstractColorCameraPtr();
		}
		if (left_color_camera_)
		{
			/// Read camera properties of range tof sensor
			int camera_index = 1;
			ipa_CameraSensors::t_cameraProperty cameraProperty;
			cameraProperty.propertyID = ipa_CameraSensors::PROP_CAMERA_RESOLUTION;
			left_color_camera_->GetProperty(&cameraProperty);
			int color_sensor_width = cameraProperty.cameraResolution.xResolution;
			int color_sensor_height = cameraProperty.cameraResolution.yResolution;
			cv::Size color_image_size(color_sensor_width, color_sensor_height);
			
			/// Setup camera toolbox
			ipa_CameraSensors::CameraSensorToolboxPtr color_sensor_toolbox = ipa_CameraSensors::CreateCameraSensorToolbox();
			color_sensor_toolbox->Init(config_directory_, left_color_camera_->GetCameraType(), camera_index, color_image_size);
	
			cv::Mat d = color_sensor_toolbox->GetDistortionParameters(left_color_camera_intrinsic_type_, left_color_camera_intrinsic_id_);
			left_color_camera_info_msg_.D[0] = d.at<double>(0, 0);
			left_color_camera_info_msg_.D[1] = d.at<double>(0, 1);
			left_color_camera_info_msg_.D[2] = d.at<double>(0, 2);
			left_color_camera_info_msg_.D[3] = d.at<double>(0, 3);
			left_color_camera_info_msg_.D[4] = 0;
	
			cv::Mat k = color_sensor_toolbox->GetIntrinsicMatrix(left_color_camera_intrinsic_type_, left_color_camera_intrinsic_id_);
			left_color_camera_info_msg_.K[0] = k.at<double>(0, 0);
			left_color_camera_info_msg_.K[1] = k.at<double>(0, 1);
			left_color_camera_info_msg_.K[2] = k.at<double>(0, 2);
			left_color_camera_info_msg_.K[3] = k.at<double>(1, 0);
			left_color_camera_info_msg_.K[4] = k.at<double>(1, 1);
			left_color_camera_info_msg_.K[5] = k.at<double>(1, 2);
			left_color_camera_info_msg_.K[6] = k.at<double>(2, 0);
			left_color_camera_info_msg_.K[7] = k.at<double>(2, 1);
			left_color_camera_info_msg_.K[8] = k.at<double>(2, 2);
	
			left_color_camera_info_msg_.width = color_sensor_width;		
			left_color_camera_info_msg_.height = color_sensor_height;		
		}

		if (right_color_camera_ && (right_color_camera_->Init(config_directory_, 0) & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Initialization of right camera (0) failed");
			right_color_camera_ = AbstractColorCameraPtr();
		}

		if (right_color_camera_ && (right_color_camera_->Open() & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Opening right color camera (0) failed");
			right_color_camera_ = AbstractColorCameraPtr();
		}
		if (right_color_camera_)
		{
			int camera_index = 0;
			/// Read camera properties of range tof sensor
			ipa_CameraSensors::t_cameraProperty cameraProperty;
			cameraProperty.propertyID = ipa_CameraSensors::PROP_CAMERA_RESOLUTION;
			right_color_camera_->GetProperty(&cameraProperty);
			int color_sensor_width = cameraProperty.cameraResolution.xResolution;
			int color_sensor_height = cameraProperty.cameraResolution.yResolution;
			cv::Size color_image_size(color_sensor_width, color_sensor_height);

			/// Setup camera toolbox
			ipa_CameraSensors::CameraSensorToolboxPtr color_sensor_toolbox = ipa_CameraSensors::CreateCameraSensorToolbox();
			color_sensor_toolbox->Init(config_directory_, left_color_camera_->GetCameraType(), camera_index, color_image_size);
	
			cv::Mat d = color_sensor_toolbox->GetDistortionParameters(right_color_camera_intrinsic_type_, right_color_camera_intrinsic_id_);
			right_color_camera_info_msg_.D[0] = d.at<double>(0, 0);
			right_color_camera_info_msg_.D[1] = d.at<double>(0, 1);
			right_color_camera_info_msg_.D[2] = d.at<double>(0, 2);
			right_color_camera_info_msg_.D[3] = d.at<double>(0, 3);
			right_color_camera_info_msg_.D[4] = 0;
	
			cv::Mat k = color_sensor_toolbox->GetIntrinsicMatrix(right_color_camera_intrinsic_type_, right_color_camera_intrinsic_id_);
			right_color_camera_info_msg_.K[0] = k.at<double>(0, 0);
			right_color_camera_info_msg_.K[1] = k.at<double>(0, 1);
			right_color_camera_info_msg_.K[2] = k.at<double>(0, 2);
			right_color_camera_info_msg_.K[3] = k.at<double>(1, 0);
			right_color_camera_info_msg_.K[4] = k.at<double>(1, 1);
			right_color_camera_info_msg_.K[5] = k.at<double>(1, 2);
			right_color_camera_info_msg_.K[6] = k.at<double>(2, 0);
			right_color_camera_info_msg_.K[7] = k.at<double>(2, 1);
			right_color_camera_info_msg_.K[8] = k.at<double>(2, 2);
	
			right_color_camera_info_msg_.width = color_sensor_width;		
			right_color_camera_info_msg_.height = color_sensor_height;		
		}

		if (tof_camera_ && (tof_camera_->Init(config_directory_) & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Initialization of tof camera (0) failed");
			tof_camera_ = AbstractRangeImagingSensorPtr();
		}

		if (tof_camera_ && (tof_camera_->Open() & ipa_CameraSensors::RET_FAILED))
		{
			ROS_WARN("[all_cameras] Opening tof camera (0) failed");
			tof_camera_ = AbstractRangeImagingSensorPtr();
		}
		if (tof_camera_)
		{
			int camera_index = 0;
			/// Read camera properties of range tof sensor
			ipa_CameraSensors::t_cameraProperty cameraProperty;
			cameraProperty.propertyID = ipa_CameraSensors::PROP_CAMERA_RESOLUTION;
			tof_camera_->GetProperty(&cameraProperty);
			int range_sensor_width = cameraProperty.cameraResolution.xResolution;
			int range_sensor_height = cameraProperty.cameraResolution.yResolution;
			cv::Size rangeImageSize(range_sensor_width, range_sensor_height);
	
			/// Setup camera toolbox
			ipa_CameraSensors::CameraSensorToolboxPtr tof_sensor_toolbox = ipa_CameraSensors::CreateCameraSensorToolbox();
			tof_sensor_toolbox->Init(config_directory_, tof_camera_->GetCameraType(), camera_index, rangeImageSize);

			cv::Mat intrinsic_mat = tof_sensor_toolbox->GetIntrinsicMatrix(tof_camera_intrinsic_type_, tof_camera_intrinsic_id_);
			cv::Mat distortion_map_X = tof_sensor_toolbox->GetDistortionMapX(tof_camera_intrinsic_type_, tof_camera_intrinsic_id_);
			cv::Mat distortion_map_Y = tof_sensor_toolbox->GetDistortionMapY(tof_camera_intrinsic_type_, tof_camera_intrinsic_id_);
			tof_camera_->SetIntrinsics(intrinsic_mat, distortion_map_X, distortion_map_Y);

			cv::Mat d = tof_sensor_toolbox->GetDistortionParameters(tof_camera_intrinsic_type_, tof_camera_intrinsic_id_);
			tof_camera_info_msg_.D[0] = d.at<double>(0, 0);
			tof_camera_info_msg_.D[1] = d.at<double>(0, 1);
			tof_camera_info_msg_.D[2] = d.at<double>(0, 2);
			tof_camera_info_msg_.D[3] = d.at<double>(0, 3);
			tof_camera_info_msg_.D[4] = 0;
	
			cv::Mat k = tof_sensor_toolbox->GetIntrinsicMatrix(tof_camera_intrinsic_type_, tof_camera_intrinsic_id_);
			tof_camera_info_msg_.K[0] = k.at<double>(0, 0);
			tof_camera_info_msg_.K[1] = k.at<double>(0, 1);
			tof_camera_info_msg_.K[2] = k.at<double>(0, 2);
			tof_camera_info_msg_.K[3] = k.at<double>(1, 0);
			tof_camera_info_msg_.K[4] = k.at<double>(1, 1);
			tof_camera_info_msg_.K[5] = k.at<double>(1, 2);
			tof_camera_info_msg_.K[6] = k.at<double>(2, 0);
			tof_camera_info_msg_.K[7] = k.at<double>(2, 1);
			tof_camera_info_msg_.K[8] = k.at<double>(2, 2);

			tof_camera_info_msg_.width = range_sensor_width;		
			tof_camera_info_msg_.height = range_sensor_height;		
		}
	
		/// Topics and Services to publish
		if (left_color_camera_) 
		{
			// Adapt name according to camera type
			left_color_image_publisher_ = image_transport_.advertiseCamera(left_color_camera_ns_ + "/left/image_color", 1);
			left_color_camera_info_service_ = node_handle_.advertiseService(left_color_camera_ns_ + "/left/set_camera_info", &CobAllCamerasNode::setCameraInfo, this);
		}
		if (right_color_camera_)
		{
			// Adapt name according to camera type
			right_color_image_publisher_ = image_transport_.advertiseCamera(right_color_camera_ns_ + "/right/image_color", 1);
			right_color_camera_info_service_ = node_handle_.advertiseService(right_color_camera_ns_ + "/right/set_camera_info", &CobAllCamerasNode::setCameraInfo, this);
		}
		if (tof_camera_)
		{
			grey_tof_image_publisher_ = image_transport_.advertiseCamera(tof_camera_ns_ + "/image_grey", 1);
			xyz_tof_image_publisher_ = image_transport_.advertiseCamera(tof_camera_ns_ + "/image_xyz", 1);
			tof_camera_info_service_ = node_handle_.advertiseService(tof_camera_ns_ + "/set_camera_info", &CobAllCamerasNode::setCameraInfo, this);
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
		// Set maximal spinning rate
		ros::Rate rate(30);
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
	
			// Acquire right color image
			if (right_color_camera_)
			{
				//ROS_INFO("[all_cameras] RIGHT");
				/// Acquire new image
				if (right_color_camera_->GetColorImage(&right_color_image_8U3_, false) & ipa_Utils::RET_FAILED)
				{
					ROS_ERROR("[all_cameras] Right color image acquisition failed");
					break;
				}

				try
		  		{
					IplImage img = right_color_image_8U3_;
					right_color_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&img, "bgr8"));
				}
				catch (sensor_msgs::CvBridgeException error)
				{
					ROS_ERROR("[all_cameras] Could not convert right IplImage to ROS message");
					break;
				}
				right_color_image_msg.header.stamp = now;    
		
				right_color_image_info = right_color_camera_info_msg_;
				right_color_image_info.width = right_color_image_8U3_.cols;
				right_color_image_info.height = right_color_image_8U3_.rows;
				right_color_image_info.header.stamp = now;
	
				right_color_image_publisher_.publish(right_color_image_msg, right_color_image_info);
			}
		
			// Acquire left color image
			if (left_color_camera_)
			{
				//ROS_INFO("[all_cameras] LEFT");
		
				/// Acquire new image
				if (left_color_camera_->GetColorImage(&left_color_image_8U3_, false) & ipa_Utils::RET_FAILED)
				{
					ROS_ERROR("[all_cameras] Left color image acquisition failed");
					break;
				}

				try
		  		{
					IplImage img = left_color_image_8U3_;
					left_color_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&img, "bgr8"));
				}
				catch (sensor_msgs::CvBridgeException error)
				{
					ROS_ERROR("[all_cameras] Could not convert left IplImage to ROS message");
					break;
				}
				left_color_image_msg.header.stamp = now;    
		
				left_color_image_info = left_color_camera_info_msg_;
				left_color_image_info.width = left_color_image_8U3_.cols;
				left_color_image_info.height = left_color_image_8U3_.rows;
				left_color_image_info.header.stamp = now;
	
				left_color_image_publisher_.publish(left_color_image_msg, left_color_image_info);
			}
	
			// Acquire image from tof camera	
			if (tof_camera_)
			{
				//ROS_INFO("[all_cameras] TOF");
				if(tof_camera_->AcquireImages(0, &grey_tof_image_32F1_, &xyz_tof_image_32F3_, false, false, ipa_CameraSensors::INTENSITY) & ipa_Utils::RET_FAILED)
				{
					ROS_ERROR("[all_cameras] Tof image acquisition failed");
					tof_camera_->Close();
		      tof_camera_ = AbstractRangeImagingSensorPtr();
					break;	
				}
	
				try
		                {
					IplImage grey_img = grey_tof_image_32F1_; 
					IplImage xyz_img = xyz_tof_image_32F3_; 
		                        xyz_tof_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&xyz_img, "passthrough"));
		                        grey_tof_image_msg = *(sensor_msgs::CvBridge::cvToImgMsg(&grey_img, "passthrough"));
		                }
		                catch (sensor_msgs::CvBridgeException error)
		                {
		                        ROS_ERROR("[all_cameras] Could not convert tof IplImage to ROS message");
					break;
		                }
	
				xyz_tof_image_msg.header.stamp = now;    
				grey_tof_image_msg.header.stamp = now;    
		
				tof_image_info = tof_camera_info_msg_;
				tof_image_info.width = grey_tof_image_32F1_.cols;
				tof_image_info.height = grey_tof_image_32F1_.rows;
				tof_image_info.header.stamp = now;
				
				grey_tof_image_publisher_.publish(grey_tof_image_msg, tof_image_info);
				xyz_tof_image_publisher_.publish(xyz_tof_image_msg, tof_image_info);
			}
			
			ros::spinOnce();
			rate.sleep();
	
		} // END while-loop
	}

	bool loadParameters()
	{
		std::string tmp_string = "NULL";

		/// Parameters are set within the launch file
		if (node_handle_.getParam("all_cameras/configuration_files", config_directory_) == false)
		{
			ROS_ERROR("Path to xml configuration file not specified");
			return false;
		}

		ROS_INFO("Configuration directory: %s", config_directory_.c_str());

		// Color camera type
		if (node_handle_.getParam("all_cameras/color_camera_type", tmp_string) == false)
		{
			ROS_ERROR("[all_cameras] Color camera type not specified");
			return false;
		}
		if (tmp_string == "CAM_AVTPIKE")
		{
			right_color_camera_ = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();
			left_color_camera_ = ipa_CameraSensors::CreateColorCamera_AVTPikeCam();

			left_color_camera_ns_ = "pike_145C";
			right_color_camera_ns_ = "pike_145C";
		}
		if (tmp_string == "CAM_VIRTUAL") 
		{
			right_color_camera_ = ipa_CameraSensors::CreateColorCamera_VirtualCam();
			left_color_camera_ = ipa_CameraSensors::CreateColorCamera_VirtualCam();
			left_color_camera_ns_ = "pike_145C";
			right_color_camera_ns_ = "pike_145C";
		}
		else if (tmp_string == "CAM_PROSILICA") ROS_ERROR("[all_cameras] Color camera type not CAM_PROSILICA not yet implemented");
		else
		{
			std::string str = "[all_cameras] Camera type '" + tmp_string + "' unknown, try 'CAM_AVTPIKE' or 'CAM_PROSILICA'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}

		ROS_INFO("Color camera type: %s", tmp_string.c_str());

		// Tof camera type
		if (node_handle_.getParam("all_cameras/tof_camera_type", tmp_string) == false)
		{
			ROS_ERROR("[all_cameras] tof camera type not specified");
			return false;
		}
		if (tmp_string == "CAM_SWISSRANGER") 
		{
			tof_camera_ = ipa_CameraSensors::CreateRangeImagingSensor_Swissranger();
			tof_camera_ns_ = "sr4000";
		}
		else if (tmp_string == "CAM_VIRTUAL") 
		{
			tof_camera_ = ipa_CameraSensors::CreateRangeImagingSensor_VirtualCam();
			tof_camera_ns_ = "sr4000";
		}
		else
		{
			std::string str = "[all_cameras] Camera type '" + tmp_string + "' unknown, try 'CAM_SWISSRANGER'";
			ROS_ERROR("%s", str.c_str());
		}
		
		ROS_INFO("Tof camera type: %s", tmp_string.c_str());

		// There are several intrinsic matrices, optimized to different cameras
		// Here, we specified the desired intrinsic matrix for each camera
		if (node_handle_.getParam("all_cameras/left_color_camera_intrinsic_type", tmp_string) == false)
		{
			ROS_ERROR("[all_cameras] Intrinsic camera type for left color camera not specified");
			return false;
		}
		if (tmp_string == "CAM_AVTPIKE")
		{
			left_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_AVTPIKE;
		}
		else if (tmp_string == "CAM_PROSILICA")
		{
			left_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_PROSILICA;
		} 
		else if (tmp_string == "CAM_SWISSRANGER")
		{
			left_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_SWISSRANGER;
		} 
		else if (tmp_string == "CAM_VIRTUALRANGE")
		{
			left_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_VIRTUALRANGE;
		} 
		else if (tmp_string == "CAM_VIRTUALCOLOR")
		{
			left_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_VIRTUALCOLOR;
		} 
		else
		{
			std::string str = "[all_cameras] Camera type '" + tmp_string + "' for intrinsics  unknown, try 'CAM_AVTPIKE','CAM_PROSILICA' or 'CAM_SWISSRANGER'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}
		if (node_handle_.getParam("all_cameras/left_color_camera_intrinsic_id", left_color_camera_intrinsic_id_) == false)
		{
			ROS_ERROR("[all_cameras] Intrinsic camera id for left color camera not specified");
			return false;
		}

		
		ROS_INFO("Intrinsic for left color camera: %s_%d", tmp_string.c_str(), left_color_camera_intrinsic_id_);

		// There are several intrinsic matrices, optimized to different cameras
		// Here, we specified the desired intrinsic matrix for each camera
		if (node_handle_.getParam("all_cameras/right_color_camera_intrinsic_type", tmp_string) == false)
		{
			ROS_ERROR("[all_cameras] Intrinsic camera type for right color camera not specified");
			return false;
		}
		if (tmp_string == "CAM_AVTPIKE")
		{
			right_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_AVTPIKE;
		}
		else if (tmp_string == "CAM_PROSILICA")
		{
			right_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_PROSILICA;
		} 
		else if (tmp_string == "CAM_SWISSRANGER")
		{
			right_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_SWISSRANGER;
		} 
		else if (tmp_string == "CAM_VIRTUALRANGE")
		{
			right_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_VIRTUALRANGE;
		} 
		else if (tmp_string == "CAM_VIRTUALCOLOR")
		{
			right_color_camera_intrinsic_type_ = ipa_CameraSensors::CAM_VIRTUALCOLOR;
		} 
		else
		{
			std::string str = "[all_cameras] Camera type '" + tmp_string + "' for intrinsics  unknown, try 'CAM_AVTPIKE','CAM_PROSILICA' or 'CAM_SWISSRANGER'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}
		if (node_handle_.getParam("all_cameras/right_color_camera_intrinsic_id", right_color_camera_intrinsic_id_) == false)
		{
			ROS_ERROR("[all_cameras] Intrinsic camera id for right color camera not specified");
			return false;
		}

		ROS_INFO("Intrinsic for right color camera: %s_%d", tmp_string.c_str(), right_color_camera_intrinsic_id_);

		// There are several intrinsic matrices, optimized to different cameras
		// Here, we specified the desired intrinsic matrix for each camera
		if (node_handle_.getParam("all_cameras/tof_camera_intrinsic_type", tmp_string) == false)
		{
			ROS_ERROR("[all_cameras] Intrinsic camera type for tof camera not specified");
			return false;
		}
		if (tmp_string == "CAM_AVTPIKE")
		{
			tof_camera_intrinsic_type_ = ipa_CameraSensors::CAM_AVTPIKE;
		}
		else if (tmp_string == "CAM_PROSILICA")
		{
			tof_camera_intrinsic_type_ = ipa_CameraSensors::CAM_PROSILICA;
		} 
		else if (tmp_string == "CAM_SWISSRANGER")
		{
			tof_camera_intrinsic_type_ = ipa_CameraSensors::CAM_SWISSRANGER;
		} 
		else if (tmp_string == "CAM_VIRTUALRANGE")
		{
			tof_camera_intrinsic_type_ = ipa_CameraSensors::CAM_VIRTUALRANGE;
		} 
		else if (tmp_string == "CAM_VIRTUALCOLOR")
		{
			tof_camera_intrinsic_type_ = ipa_CameraSensors::CAM_VIRTUALCOLOR;
		} 
		else
		{
			std::string str = "[all_cameras] Camera type '" + tmp_string + "' for intrinsics  unknown, try 'CAM_AVTPIKE','CAM_PROSILICA' or 'CAM_SWISSRANGER'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}
		if (node_handle_.getParam("all_cameras/tof_camera_intrinsic_id", tof_camera_intrinsic_id_) == false)
		{
			ROS_ERROR("[all_cameras] Intrinsic camera id for tof camera not specified");
			return false;
		}

		ROS_INFO("Intrinsic for tof camera: %s_%d", tmp_string.c_str(), tof_camera_intrinsic_id_);
	
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
	CobAllCamerasNode camera_node(nh);

	/// initialize camera node
	if (camera_node.init() == false)
	{
		ROS_ERROR("[all_cameras] Node initialization FAILED. Terminating");
		return 0;
	}
	else
	{
		ROS_INFO("[all_cameras] Node initialization OK. Enter spinning");
	}

	camera_node.spin();
	
	return 0;
}
