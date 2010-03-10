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
#include <cob_vision_utils/GlobalDefines.h>
#include <cob_vision_utils/OpenCVUtils.h>

using namespace ipa_CameraSensors;

class CobTofCameraNode
{
private:
        ros::NodeHandle m_NodeHandle;	///< Node handle

	image_transport::ImageTransport m_ImageTransport;	///< Image transport instance
	image_transport::Publisher m_XYZImagePublisher;		///< Publishes xyz image data
	image_transport::Publisher m_GrayImagePublisher;	///< Publishes gray image data

        sensor_msgs::Image m_XYZImageMessage;      ///< ROS image message
        sensor_msgs::Image m_GrayImageMessage;      ///< ROS image message
        sensor_msgs::CameraInfo m_CameraInfoMessage;    ///< ROS camera information message (e.g. holding intrinsic parameters)

        ros::ServiceServer m_CameraInfoService;		///< Service to set/modify camera parameters

        AbstractRangeImagingSensor* m_TofCamera;     ///< Time-of-flight camera instance

        IplImage* m_XYZImage32F3;	/// OpenCV image holding the point cloud
        IplImage* m_GrayImage32F1;	/// OpenCV image holding the amplitude values

public:
	/// Constructor.
        CobTofCameraNode(const ros::NodeHandle& node_handle)
        : m_NodeHandle(node_handle),
	  m_ImageTransport(node_handle),
          m_TofCamera(0),
          m_XYZImage32F3(0),
          m_GrayImage32F1(0)
        {
                /// Void
        }
	
	/// Destructor
	~CobTofCameraNode()
        {
                m_TofCamera->Close();
                ipa_CameraSensors::ReleaseRangeImagingSensor(m_TofCamera);

		if (m_XYZImage32F3) cvReleaseImage(&m_XYZImage32F3);
		if (m_GrayImage32F1) cvReleaseImage(&m_GrayImage32F1);
        }

        /// Initializes and opens the time-of-flight camera sensor.
	/// @return <code>false</code> on failure, <code>true</code> otherwise
        bool Init()
        {
		int cameraIndex = 0;
                std::string directory = "NULL/";
                
		/// Parameters are set within the launch file
                if (m_NodeHandle.getParam("tof_camera/configuration_files", directory) == false)
                {
                        ROS_ERROR("Path to xml configuration for color camera not specified");
                        return false;
                }

		m_TofCamera = ipa_CameraSensors::CreateRangeImagingSensor_Swissranger();

		if (m_TofCamera->Init(directory) & ipa_CameraSensors::RET_FAILED)
		{

			std::stringstream ss;
                        ss << "Initialization of tof camera ";
                        ss << cameraIndex;
                        ss << " failed";
                        ROS_ERROR("%s", ss.str().c_str());
                        m_TofCamera = 0;
                        return false;
		}
	
		if (m_TofCamera->Open() & ipa_CameraSensors::RET_FAILED)
		{
			std::stringstream ss;
                        ss << "Could not open tof camera ";
                        ss << cameraIndex;
                        ROS_ERROR("%s", ss.str().c_str());
                        m_TofCamera = 0;
                        return false;
		}

                /// Advertise service for other nodes to set intrinsic calibration parameters
                m_CameraInfoService = m_NodeHandle.advertiseService("set_camera_info", &CobTofCameraNode::SetCameraInfo, this);
		m_XYZImagePublisher = m_ImageTransport.advertise("xyz_data", 1);
		m_GrayImagePublisher = m_ImageTransport.advertise("gray_data", 1);

		return true;
	}

	/// Enables the user to modify camera parameters.
        /// @param req Requested camera parameters
        /// @param rsp Response, telling if requested parameters have been set
        /// @return <code>True</code>
        bool SetCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
                        sensor_msgs::SetCameraInfo::Response& rsp)
        {
                /// TODO: Enable the setting of intrinsic parameters
                m_CameraInfoMessage = req.camera_info;

                rsp.success = false;
                rsp.status_message = "Setting camera parameters through ROS not implemented";

                return true;
        }

        /// Continuously advertises xyz and gray
	bool Spin()
        {
		sensor_msgs::Image XYZImageMsg;
		sensor_msgs::Image grayImageMsg;

		ros::Rate rate(10);
		while(m_NodeHandle.ok())
		{
	                /// Release previously acquired IplImage 
	                if (m_XYZImage32F3)
	                {
	                        cvReleaseImage(&m_XYZImage32F3);
	                        m_XYZImage32F3 = 0;
	                }
	
	                if (m_GrayImage32F1)
	                {
	                        cvReleaseImage(&m_GrayImage32F1);
	                        m_GrayImage32F1 = 0;
	                }
	
			if(m_TofCamera->AcquireImages2(0, &m_GrayImage32F1, &m_XYZImage32F3, false, false, ipa_CameraSensors::AMPLITUDE) & ipa_Utils::RET_FAILED)
			{
				ROS_ERROR("Tof image acquisition failed");
	                        return false;	
			}
			
			try
	                {
	                        XYZImageMsg = *(sensor_msgs::CvBridge::cvToImgMsg(m_XYZImage32F3, "passthrough"));
	                }
	                catch (sensor_msgs::CvBridgeException error)
	                {
	                        ROS_ERROR("Could not convert 32bit xyz IplImage to ROS message");
				return false;
	                }
	
			try
	                {
	                        grayImageMsg = *(sensor_msgs::CvBridge::cvToImgMsg(m_GrayImage32F1, "passthrough"));
	                }
	                catch (sensor_msgs::CvBridgeException error)
	                {
	                        ROS_ERROR("Could not convert 32bit gray IplImage to ROS message");
				return false;
	                }
	
	                /// Set time stamp
			ros::Time now = ros::Time::now();
	                XYZImageMsg.header.stamp = now;
	                grayImageMsg.header.stamp = now;

			/// publish message
			m_XYZImagePublisher.publish(XYZImageMsg);
			m_GrayImagePublisher.publish(grayImageMsg);

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
        CobTofCameraNode cameraNode(nh);

        /// Initialize camera node
        if (!cameraNode.Init()) return 0;

	cameraNode.Spin();

        return 0;
}
