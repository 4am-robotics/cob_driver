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

// ROS includes
#include <ros/ros.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>

// ROS message includes
#include <sensor_msgs/Image.h>

// External includes
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cob_vision_utils/OpenCVUtils.h>

//####################
//#### node class ####
class CobTofCameraViewerNode
{
private:
	ros::NodeHandle m_NodeHandle;   ///< Node handle

	image_transport::ImageTransport m_ImageTransport;       ///< Image transport instance
	image_transport::Subscriber m_XYZImageSubscriber;	///< Subscribes to xyz image data
        image_transport::Subscriber m_GrayImageSubscriber;	///< Subscribes to gray image data

	sensor_msgs::CvBridge m_CvBridge0;
	sensor_msgs::CvBridge m_CvBridge1;
	
	IplImage* m_XYZImage32F3;	/// OpenCV image holding the 32bit point cloud
	IplImage* m_XYZImage8U3;	/// OpenCV image holding the transformed 8bit RGB point cloud
        IplImage* m_GrayImage32F1;	/// OpenCV image holding the 32bit amplitude values
        IplImage* m_GrayImage8U3;	/// OpenCV image holding the transformed 8bit RGB amplitude values

public:
	/// Constructor.
	/// @param node_handle Node handle instance
        CobTofCameraViewerNode(const ros::NodeHandle& node_handle)
        : m_NodeHandle(node_handle),
          m_ImageTransport(node_handle),
          m_XYZImage32F3(0),
          m_XYZImage8U3(0),
          m_GrayImage32F1(0),
          m_GrayImage8U3(0)
        {
                /// Void
        }

	/// Destructor.
        ~CobTofCameraViewerNode()
        {
		/// Do not release <code>m_GrayImage32F3</code>
		/// Do not release <code>m_XYZImage32F3</code>
		/// Image allocation is managed by Cv_Bridge object 
                if (m_XYZImage8U3) cvReleaseImage(&m_XYZImage8U3);
                if (m_GrayImage8U3) cvReleaseImage(&m_GrayImage8U3);

		if(cvGetWindowHandle("z data"))cvDestroyWindow("z data");
		if(cvGetWindowHandle("gray data"))cvDestroyWindow("gray data");
        }

	/// Initialize tof camera viewer.
        /// @return <code>false</code> on failure, <code>true</code> otherwise
	bool Init()
	{
		/// Create viewer windows
		cvStartWindowThread();
		cvNamedWindow("z data");		
		cvNamedWindow("gray data");		

		m_XYZImageSubscriber = m_ImageTransport.subscribe("camera/xyz_data", 1, &CobTofCameraViewerNode::XYZImageCallback, this);
		m_GrayImageSubscriber = m_ImageTransport.subscribe("camera/gray_data", 1, &CobTofCameraViewerNode::GrayImageCallback, this);

		return true;
	}

        /// Topic callback functions. 
        /// Function will be called when a new message arrives on a topic.
	/// @param grayImageMsg The gray values of point cloud, saved in a 32bit, 1 channel OpenCV IplImage
        void GrayImageCallback(const sensor_msgs::ImageConstPtr& grayImageMsg)
	{
		/// Do not release <code>m_GrayImage32F3</code>
		/// Image allocation is managed by Cv_Bridge object 

		try
		{
			m_GrayImage32F1 = m_CvBridge0.imgMsgToCv(grayImageMsg, "passthrough");

                	if (m_GrayImage8U3 == 0)
	                {
				m_GrayImage8U3 = cvCreateImage(cvGetSize(m_GrayImage32F1), IPL_DEPTH_8U, 3);
			}

			ipa_Utils::ConvertToShowImage(m_GrayImage32F1, m_GrayImage8U3, 1, 0, 10000);
			cvShowImage("gray data", m_GrayImage8U3);
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("Could not convert from '%s' to '32FC1'.", grayImageMsg->encoding.c_str());
		}
	}

        /// Topic callback functions. 
        /// Function will be called when a new message arrives on a topic.
	/// @param xyzImageMsg The point cloud, saved in a 32bit, 3 channel OpenCV IplImage
        void XYZImageCallback(const sensor_msgs::ImageConstPtr& xyzImageMsg)
        {
		/// Do not release <code>m_XYZImage32F3</code>
		/// Image allocation is managed by Cv_Bridge object 

		try
		{
			m_XYZImage32F3 = m_CvBridge1.imgMsgToCv(xyzImageMsg, "passthrough");
			
                	if (m_XYZImage8U3 == 0)
	                {
				m_XYZImage8U3 = cvCreateImage(cvGetSize(m_XYZImage32F3), IPL_DEPTH_8U, 3);
			}

			ipa_Utils::ConvertToShowImage(m_XYZImage32F3, m_XYZImage8U3, 3);
			cvShowImage("z data", m_XYZImage8U3);
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("Could not convert from '%s' to '32FC1'.", xyzImageMsg->encoding.c_str());
		}

        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
        /// initialize ROS, spezify name of node
        ros::init(argc, argv, "tof_camera_viewer");

        /// Create a handle for this node, initialize node
        ros::NodeHandle nh;

        /// Create camera node class instance   
        CobTofCameraViewerNode cameraViewerNode(nh);

        /// Initialize camera node
        if (!cameraViewerNode.Init()) return 0;

        ros::spin();

        return 0;
}


/*
 int numPoints = image->width*image->height;
 sensor_msgs::PointCloud pc_msg;
 pc_msg.header.stamp = ros::Time::now(); 
 pc_msg.points.resize(numPoints);
               
 /// Convert OpenCV image to PointCloud
 for (int i = 0; i < numPoints; i++)
 {
 msg.points[i].x = ((float*)image->imageData)[3*i + 0];
 msg.points[i].y = ((float*)image->imageData)[3*i + 1];
 msg.points[i].z = ((float*)image->imageData)[3*i + 2];
 }
*/
/*
IplImage* image = cvCreateImage(cvSize(176, 144), IPL_DEPTH_32F, 3);
 for (unsigned int i=0; i<msg->points.size(); i++)
 {
 ((float*)image->imageData)[3*i+0] = (msg->points[i]).x;
 ((float*)image->imageData)[3*i+1] = (msg->points[i]).y;
 ((float*)image->imageData)[3*i+2] = (msg->points[i]).z;
 }
 IplImage* image_show = cvCreateImage(cvSize(176, 144), IPL_DEPTH_8U, 3);
 ipa_Utils::ConvertToShowImage(image, image_show, 3);
*/
