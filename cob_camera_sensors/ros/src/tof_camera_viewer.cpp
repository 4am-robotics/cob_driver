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
#include <cob_vision_ipa_utils/PointCloudRenderer.h>

// ROS message includes
#include <sensor_msgs/Image.h>

// External includes
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cob_vision_utils/OpenCVUtils.h>
#include <sstream>

//####################
//#### node class ####
class CobTofCameraViewerNode
{
private:
	ros::NodeHandle m_NodeHandle;   ///< Node handle

	image_transport::ImageTransport image_transport_;       ///< Image transport instance
	image_transport::Subscriber xyz_image_subscriber_;	///< Subscribes to xyz image data
	image_transport::Subscriber grey_image_subscriber_;	///< Subscribes to gray image data

	sensor_msgs::CvBridge cv_bridge_0_;
	sensor_msgs::CvBridge cv_bridge_1_;
	
	IplImage* xyz_image_32F3_;	/// OpenCV image holding the 32bit point cloud
	IplImage* xyz_image_8U3_;	/// OpenCV image holding the transformed 8bit RGB point cloud
	IplImage* grey_image_32F1_;	/// OpenCV image holding the 32bit amplitude values
	IplImage* grey_image_8U3_;	/// OpenCV image holding the transformed 8bit RGB amplitude values

	int grey_image_counter_; 
	bool use_opengl_;

public:
	/// Constructor.
	/// @param node_handle Node handle instance
        CobTofCameraViewerNode(const ros::NodeHandle& node_handle, bool use_opengl=false)
        : m_NodeHandle(node_handle),
          image_transport_(node_handle),
          xyz_image_32F3_(0),
          xyz_image_8U3_(0),
          grey_image_32F1_(0),
          grey_image_8U3_(0),
		  grey_image_counter_(0)
        {
        	use_opengl_ = use_opengl;
        }

	/// Destructor.
        ~CobTofCameraViewerNode()
        {
		/// Do not release <code>m_GrayImage32F3</code>
		/// Do not release <code>xyz_image_32F3_</code>
		/// Image allocation is managed by Cv_Bridge object 
			if (xyz_image_8U3_) cvReleaseImage(&xyz_image_8U3_);
			if (grey_image_8U3_) cvReleaseImage(&grey_image_8U3_);

			if(cvGetWindowHandle("z data"))cvDestroyWindow("z data");
			if(cvGetWindowHandle("gray data"))cvDestroyWindow("gray data");
			if (use_opengl_)
			{
				PointCloudRenderer::Exit();
			}
        }

	/// initialize tof camera viewer.
        /// @return <code>false</code> on failure, <code>true</code> otherwise
	bool init()
	{
		/// Create viewer windows
		cvStartWindowThread();
		cvNamedWindow("z data");		
		cvNamedWindow("gray data");		

		xyz_image_subscriber_ = image_transport_.subscribe("camera/xyz_tof_data", 1, &CobTofCameraViewerNode::xyzImageCallback, this);
		grey_image_subscriber_ = image_transport_.subscribe("camera/grey_tof_data", 1, &CobTofCameraViewerNode::greyImageCallback, this);

        if (use_opengl_)
        {
        	PointCloudRenderer::Init();
        	//PointCloudRenderer::SetKeyboardPointer(&m_GlKey);
        	PointCloudRenderer::Run();
        }

		return true;
	}

	/// Topic callback functions. 
	/// Function will be called when a new message arrives on a topic.
	/// @param grey_image_msg The gray values of point cloud, saved in a 32bit, 1 channel OpenCV IplImage
	void greyImageCallback(const sensor_msgs::ImageConstPtr& grey_image_msg)
	{
		/// Do not release <code>m_GrayImage32F3</code>
		/// Image allocation is managed by Cv_Bridge object 

		try
		{
			grey_image_32F1_ = cv_bridge_0_.imgMsgToCv(grey_image_msg, "passthrough");

			if (grey_image_8U3_ == 0)
			{
				grey_image_8U3_ = cvCreateImage(cvGetSize(grey_image_32F1_), IPL_DEPTH_8U, 3);
			}

			ipa_Utils::ConvertToShowImage(grey_image_32F1_, grey_image_8U3_, 1, 0, 700);
			cvShowImage("gray data", grey_image_8U3_);
			int c = cvWaitKey(50);
			if (c=='s' || c==536871027)
			{
				std::stringstream ss;
				char counterBuffer [50];
				sprintf(counterBuffer, "%04d", grey_image_counter_);
				ss << "greyImage8U3_";
				ss << counterBuffer;
				ss << ".bmp";
				cvSaveImage(ss.str().c_str(),grey_image_8U3_);
				std::cout << "Image " << grey_image_counter_ << " saved." << std::endl;
				grey_image_counter_++;
			}
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("[tof_camera_viewer] Could not convert from '%s' to '32FC1'.", grey_image_msg->encoding.c_str());
		}
	}

	/// Topic callback functions. 
	/// Function will be called when a new message arrives on a topic.
	/// @param xyz_image_msg The point cloud, saved in a 32bit, 3 channel OpenCV IplImage
	void xyzImageCallback(const sensor_msgs::ImageConstPtr& xyz_image_msg)
	{
		/// Do not release <code>xyz_image_32F3_</code>
		/// Image allocation is managed by Cv_Bridge object 

		static IplImage* xyz_image_32F3_temp = 0;
		static IplImage* xyz_image_32F3_temp2 = 0;

		try
		{
			xyz_image_32F3_temp2 = xyz_image_32F3_temp;
			xyz_image_32F3_ = cv_bridge_1_.imgMsgToCv(xyz_image_msg, "passthrough");
			xyz_image_32F3_temp = cvCloneImage(xyz_image_32F3_);
			
			if (xyz_image_8U3_ == 0)
			{
				xyz_image_8U3_ = cvCreateImage(cvGetSize(xyz_image_32F3_), IPL_DEPTH_8U, 3);
			}

			ipa_Utils::ConvertToShowImage(xyz_image_32F3_, xyz_image_8U3_, 3);
			cvShowImage("z data", xyz_image_8U3_);
			if(use_opengl_)
			{
				PointCloudRenderer::SetIplImage(xyz_image_32F3_temp);
			}
			/// Now, we can savely release the memory of the previous point cloud without
			/// Disturbing the point cloud renderer
			if (xyz_image_32F3_temp2)
			{
				cvReleaseImage(&xyz_image_32F3_temp2);
				xyz_image_32F3_temp2 = 0;
			}
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("[tof_camera_viewer] Could not convert from '%s' to '32FC1'.", xyz_image_msg->encoding.c_str());
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

        bool use_opengl = false;
        nh.getParam("/tof_camera_viewer/use_opengl", use_opengl);

        /// Create camera node class instance   
        CobTofCameraViewerNode camera_viewer_node(nh, use_opengl);


        /// initialize camera node
        if (!camera_viewer_node.init()) return 0;

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
