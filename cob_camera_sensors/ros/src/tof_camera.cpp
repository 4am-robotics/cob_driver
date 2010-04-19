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
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>

// external includes
#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <cob_camera_sensors/CameraSensorToolbox.h>
#include <cob_vision_utils/GlobalDefines.h>
#include <cob_vision_utils/OpenCVUtils.h>

using namespace ipa_CameraSensors;

class CobTofCameraNode
{
private:
	ros::NodeHandle node_handle_;	///< Node handle

	image_transport::ImageTransport image_transport_;	///< Image transport instance
	image_transport::CameraPublisher xyz_image_publisher_;	///< Publishes xyz image data
	image_transport::CameraPublisher grey_image_publisher_;	///< Publishes grey image data

	sensor_msgs::CameraInfo camera_info_msg_;    ///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer camera_info_service_;		///< Service to set/modify camera parameters

	AbstractRangeImagingSensor* tof_camera_;     ///< Time-of-flight camera instance

	IplImage* xyz_image_32F3_;	/// OpenCV image holding the point cloud
	IplImage* grey_image_32F1_;	/// OpenCV image holding the amplitude values

public:
	/// Constructor.
    CobTofCameraNode(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle),
	  image_transport_(node_handle),
          tof_camera_(0),
          xyz_image_32F3_(0),
          grey_image_32F1_(0)
    {
            /// Void
    }
	
	/// Destructor
	~CobTofCameraNode()
    {
		tof_camera_->Close();
		ipa_CameraSensors::ReleaseRangeImagingSensor(tof_camera_);

		if (xyz_image_32F3_) cvReleaseImage(&xyz_image_32F3_);
		if (grey_image_32F1_) cvReleaseImage(&grey_image_32F1_);
    }

    /// Initializes and opens the time-of-flight camera sensor.
	/// @return <code>false</code> on failure, <code>true</code> otherwise
    bool init()
    {
		int camera_index = -1;
		std::string directory = "NULL/";
		std::string tmp_string = "NULL";
               
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/configuration_files", directory) == false)
		{
				ROS_ERROR("[tof_camera] Path to xml configuration for color camera not specified");
				return false;
		}

		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/camera_index", camera_index) == false)
		{
			ROS_ERROR("[tof_camera] Tof camera index (0 or 1) not specified");
			return false;
		}

		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/tof_camera_type", tmp_string) == false)
		{
			ROS_ERROR("[tof_camera] tof camera type not specified");
			return false;
		}
		if (tmp_string == "CAM_SWISSRANGER") tof_camera_ = ipa_CameraSensors::CreateRangeImagingSensor_Swissranger();
		else if (tmp_string == "CAM_PMDCAMCUBE") 
		{
			ROS_ERROR("[tof_camera] tof camera type CAM_PMDCAMCUBE not yet implemented");
			return false;
		}
		else
		{
			std::string str = "[tof_camera] Camera type '" + tmp_string + "' unknown, try 'CAM_SWISSRANGER'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}

		if (tof_camera_->Init(directory, camera_index) & ipa_CameraSensors::RET_FAILED)
		{

			std::stringstream ss;
			ss << "Initialization of tof camera ";
			ss << camera_index;
			ss << " failed";
			ROS_ERROR("[tof_camera] %s", ss.str().c_str());
			tof_camera_ = 0;
			return false;
		}
	
		if (tof_camera_->Open() & ipa_CameraSensors::RET_FAILED)
		{
			std::stringstream ss;
			ss << "Could not open tof camera ";
			ss << camera_index;
			ROS_ERROR("[tof_camera] %s", ss.str().c_str());
			tof_camera_ = 0;
			return false;
		}

		/// Read camera properties of range imaging sensor
		cameraProperty.propertyID = ipa_CameraSensors::PROP_CAMERA_RESOLUTION;
		tof_camera_->GetProperty(&cameraProperty);
		int range_sensor_width = cameraProperty.cameraResolution.xResolution;
		int range_sensor_height = cameraProperty.cameraResolution.yResolution;
		CvSize rangeImageSize = cvSize(range_sensor_width_, range_sensor_height_);

		/// Setup camera toolbox
		ipa_CameraSensors::CameraSensorToolbox* tof_sensor_toolbox = ipa_CameraSensors::CreateCameraSensorToolbox();
		tof_sensor_toolbox->Init(directory, tof_camera_->GetCameraType(), camera_index, rangeImageSize);
		tof_camera_->SetIntrinsics(tof_sensor_toolbox->GetIntrinsicMatrix(tof_camera_->GetCameraType(), camera_index),
			tof_sensor_toolbox->GetDistortionMapX(tof_camera_->GetCameraType(), camera_index),
			tof_sensor_toolbox->GetDistortionMapY(tof_camera_->GetCameraType(), camera_index));


        /// Advertise service for other nodes to set intrinsic calibration parameters
		camera_info_service_ = node_handle_.advertiseService("set_camera_info", &CobTofCameraNode::setCameraInfo, this);
		xyz_image_publisher_ = image_transport_.advertiseCamera("image_xyz", 1);
		grey_image_publisher_ = image_transport_.advertiseCamera("image_grey", 1);

		/// Release memory
		if (tof_sensor_toolbox) ipa_CameraSensors::ReleaseCameraSensorToolbox(tof_sensor_toolbox);

		return true;
	}

	/// Enables the user to modify camera parameters.
    /// @param req Requested camera parameters
    /// @param rsp Response, telling if requested parameters have been set
    /// @return <code>True</code>
    bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                    sensor_msgs::SetCameraInfo::Response& rsp)
    {
		/// TODO: Enable the setting of intrinsic parameters
		camera_info_msg_ = req.camera_info;

		rsp.success = false;
		rsp.status_message = "[tof_camera] Setting camera parameters through ROS not implemented";

		return true;
    }

    /// Continuously advertises xyz and grey
	bool spin()
    {
		sensor_msgs::Image::Ptr xyz_image_msg_ptr;
		sensor_msgs::Image::Ptr grey_image_msg_ptr;
		sensor_msgs::CameraInfo tof_image_info;

		ros::Rate rate(10);
		while(node_handle_.ok())
		{
			/// Release previously acquired IplImage 
			if (xyz_image_32F3_)
			{
					cvReleaseImage(&xyz_image_32F3_);
					xyz_image_32F3_ = 0;
			}

			if (grey_image_32F1_)
			{
					cvReleaseImage(&grey_image_32F1_);
					grey_image_32F1_ = 0;
			}
	
			if(tof_camera_->AcquireImages2(0, &grey_image_32F1_, &xyz_image_32F3_, false, false, ipa_CameraSensors::AMPLITUDE) & ipa_Utils::RET_FAILED)
			{
				ROS_ERROR("[tof_camera] Tof image acquisition failed");
	                        return false;	
			}

			try
			{
				xyz_image_msg_ptr = sensor_msgs::CvBridge::cvToImgMsg(xyz_image_32F3_, "passthrough");
			}
			catch (sensor_msgs::CvBridgeException error)
			{
				ROS_ERROR("[tof_camera] Could not convert 32bit xyz IplImage to ROS message");
				return false;
			}
	
			try
			{
				grey_image_msg_ptr = sensor_msgs::CvBridge::cvToImgMsg(grey_image_32F1_, "passthrough");
			}
			catch (sensor_msgs::CvBridgeException error)
			{
				ROS_ERROR("[tof_camera] Could not convert 32bit grey IplImage to ROS message");
				return false;
			}
	
			/// Set time stamp
			ros::Time now = ros::Time::now();
			xyz_image_msg_ptr->header.stamp = now;
			grey_image_msg_ptr->header.stamp = now;

			tof_image_info = camera_info_msg_;
			tof_image_info.width = grey_image_32F1_->width;
			tof_image_info.height = grey_image_32F1_->height;
			tof_image_info.header.stamp = now;

			/// publish message
			xyz_image_publisher_.publish(*xyz_image_msg_ptr, tof_image_info);
			grey_image_publisher_.publish(*grey_image_msg_ptr, tof_image_info);

			ros::spinOnce();
			rate.sleep();
		}
		return true;
    }
};


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, spezify name of node
    ros::init(argc, argv, "tof_camera");

    /// Create a handle for this node, initialize node
    ros::NodeHandle nh;

    /// Create camera node class instance   
    CobTofCameraNode camera_node(nh);

    /// Initialize camera node
    if (!camera_node.init()) return 0;

	camera_node.spin();

	return 0;
}
