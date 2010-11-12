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
 * ROS stack name: cob_drivers
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
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#ifdef __ROS_1_1__
	#include <message_filters/sync_policies/approximate_time.h>
	#include <message_filters/synchronizer.h>
#endif

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>

#include <cob_srvs/AcquireCalibrationImages.h>

// external includes
#include <cob_vision_utils/VisionUtils.h>

#include <opencv/highgui.h>
#include <boost/thread/mutex.hpp>

using namespace message_filters;

#ifdef __ROS_1_1__
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ThreeImageSyncPolicy;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> TwoImageSyncPolicy;
#endif

/// @class AllCameraViewer
/// This node gathers images from either 'two color cameras' or 'one
/// color camera and ont time-of-flight sensor' of 'two color cameras
/// and one time-of-flight sensor' of 'a single camera and odometry commands'
/// to create a coloured point cloud out of the provided information
class AllCameraViewer
{
private:
	ros::NodeHandle node_handle_;

	image_transport::ImageTransport image_transport_;

	// Subscriptions
	image_transport::SubscriberFilter left_color_camera_image_sub_;	///< Left color camera image topic
	image_transport::SubscriberFilter right_color_camera_image_sub_;	///< Right color camera image topic
	image_transport::SubscriberFilter tof_camera_grey_image_sub_;	///< Tof camera intensity image topic
	message_filters::Subscriber<sensor_msgs::CameraInfo> left_camera_info_sub_;	///< Left camera information service
	message_filters::Subscriber<sensor_msgs::CameraInfo> right_camera_info_sub_;	///< Right camera information service
	image_transport::Subscriber sub;
	
#ifdef __ROS_1_1__
	message_filters::Synchronizer<ThreeImageSyncPolicy> shared_sub_sync_;
	message_filters::Synchronizer<TwoImageSyncPolicy> stereo_sub_sync_;
	message_filters::Synchronizer<ThreeImageSyncPolicy> all_sub_sync_;
#else
	message_filters::TimeSynchronizer<sensor_msgs::Image,  ///< Assembles images with same timestamp
						sensor_msgs::Image, sensor_msgs::Image> shared_sub_sync_;
	message_filters::TimeSynchronizer<sensor_msgs::Image,   ///< Assembles images with same timestamp
						sensor_msgs::Image> stereo_sub_sync_;
	message_filters::TimeSynchronizer<sensor_msgs::Image, ///< Assembles images with same timestamp
						sensor_msgs::Image, sensor_msgs::Image> all_sub_sync_;
#endif

	int sub_counter_; /// Number of subscribers to topic
	int tof_image_counter_; /// Number of accumulated tog greyscale images

	sensor_msgs::Image color_image_msg_;
	sensor_msgs::Image xyz_image_msg_;
	sensor_msgs::Image confidence_mask_msg_;

	IplImage* right_color_image_8U3_;	///< Received color image of the right camera
	IplImage* left_color_image_8U3_;	///< Received color image of the left camera
	IplImage* xyz_image_32F3_;	///< Received point cloud form tof sensor
	IplImage* grey_image_32F1_;	///< Received gray values from tof sensor

	cv::Mat right_color_mat_8U3_;	///< Received color image of the right camera
	cv::Mat left_color_mat_8U3_;	///< Received color image of the left camera
	cv::Mat xyz_mat_32F3_;	///< Received point cloud form tof sensor
	cv::Mat grey_mat_32F1_;	///< Received gray values from tof sensor
	cv::Mat grey_mat_8U1_;	///< Received gray values from tof sensor
	std::vector<cv::Mat> vec_grey_mat_32F1_; ///< Accumulated tof greyscale images for noise reduction

	int image_counter_; ///< Counts the number of images saved to the hard disk

	sensor_msgs::CvBridge cv_bridge_0_; ///< Converts ROS image messages to openCV IplImages
	sensor_msgs::CvBridge cv_bridge_1_; ///< Converts ROS image messages to openCV IplImages
	sensor_msgs::CvBridge cv_bridge_2_; ///< Converts ROS image messages to openCV IplImages

	ros::ServiceServer save_camera_images_service_;

	boost::mutex m_ServiceMutex;

	std::string absolute_output_directory_path_; ///< Directory, where camera images are saved

	bool use_tof_camera_;
	bool use_left_color_camera_;
	bool use_right_color_camera_;
public:
	/// Constructor.
	AllCameraViewer(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  image_transport_(node_handle),
#ifdef __ROS_1_1__
	  shared_sub_sync_(ThreeImageSyncPolicy(3)),
	  stereo_sub_sync_(TwoImageSyncPolicy(3)),
	  all_sub_sync_(ThreeImageSyncPolicy(3)),
#else
	  shared_sub_sync_(3),
	  stereo_sub_sync_(3),
	  all_sub_sync_(3),
#endif
	  sub_counter_(0),
	  right_color_image_8U3_(0),
	  left_color_image_8U3_(0),
	  xyz_image_32F3_(0),
	  grey_image_32F1_(0)
	{
		use_tof_camera_ = true;
		use_left_color_camera_ = true;
		use_right_color_camera_ = true;
		image_counter_ = 0;
		tof_image_counter_ = 0;
	}

	/// Destructor.
	~AllCameraViewer()
	{
	} 

	/// Initialize sensor fusion node.
	/// Setup publisher of point cloud and corresponding color data,
	/// setup camera toolboxes and colored point cloud toolbox
	/// @return <code>true</code> on success, <code>false</code> otherwise
	bool init()
	{
		if (loadParameters() == false) return false;

		image_transport::SubscriberStatusCallback imgConnect    = boost::bind(&AllCameraViewer::connectCallback, this);
		image_transport::SubscriberStatusCallback imgDisconnect = boost::bind(&AllCameraViewer::disconnectCallback, this);

		save_camera_images_service_ = node_handle_.advertiseService("save_camera_images", &AllCameraViewer::saveCameraImagesServiceCallback, this);

		// Synchronize inputs of incoming image data.
		// Topic subscriptions happen on demand in the connection callback.

		// TODO: Dynamically determine, which cameras are available

		if(use_right_color_camera_ && use_tof_camera_ && !use_left_color_camera_)
		{
			ROS_INFO("[all_camera_viewer] Setting up subscribers for right color and tof camera");
			shared_sub_sync_.connectInput(right_color_camera_image_sub_, tof_camera_grey_image_sub_);
			shared_sub_sync_.registerCallback(boost::bind(&AllCameraViewer::sharedModeSrvCallback, this, _1, _2));
		}
		else if(use_right_color_camera_ && !use_tof_camera_ && use_left_color_camera_)
		{
			ROS_INFO("[all_camera_viewer] Setting up subscribers left and right color camera");
			stereo_sub_sync_.connectInput(left_color_camera_image_sub_, right_color_camera_image_sub_);
			stereo_sub_sync_.registerCallback(boost::bind(&AllCameraViewer::stereoModeSrvCallback, this, _1, _2));
		}
		else if(use_right_color_camera_ && use_tof_camera_ && use_left_color_camera_)
		{
			ROS_INFO("[all_camera_viewer] Setting up subscribers for left color, right color and tof camera");
			all_sub_sync_.connectInput(left_color_camera_image_sub_, right_color_camera_image_sub_, tof_camera_grey_image_sub_);
			all_sub_sync_.registerCallback(boost::bind(&AllCameraViewer::allModeSrvCallback, this, _1, _2, _3));
		}
		else
		{
			ROS_ERROR("[all_camera_viewer] Specified camera configuration not available");
			return false;
		}


		connectCallback();

  		ROS_INFO("[all_camera_viewer] Initializing [OK]");
		return true;
	}

	/// Subscribe to camera topics if not already done.
	void connectCallback()
	{
		if (sub_counter_ == 0) 
		{
			sub_counter_++;
			ROS_DEBUG("[all_camera_viewer] Subscribing to camera topics");

			if (use_right_color_camera_)
			{
				right_color_camera_image_sub_.subscribe(image_transport_, "right/image_color", 1);
				right_camera_info_sub_.subscribe(node_handle_, "right/camera_info", 1);
			}
			if (use_left_color_camera_)
			{
				left_color_camera_image_sub_.subscribe(image_transport_, "left/image_color", 1);
				left_camera_info_sub_.subscribe(node_handle_, "left/camera_info", 1);
			}
			if (use_tof_camera_)
			{
				tof_camera_grey_image_sub_.subscribe(image_transport_, "image_grey", 1);
			}
		}
	}

	/// Unsubscribe from camera topics if possible.
	void disconnectCallback()
	{
		sub_counter_--;
		if (sub_counter_ == 0)
		{
			ROS_DEBUG("[all_camera_viewer] Unsubscribing from camera topics");

			if (use_right_color_camera_)
			{
				right_color_camera_image_sub_.unsubscribe();
				right_camera_info_sub_.unsubscribe();
			}
			if (use_left_color_camera_)
			{
				left_color_camera_image_sub_.unsubscribe();
				left_camera_info_sub_.unsubscribe();
			}
			if (use_tof_camera_)
			{
				tof_camera_grey_image_sub_.unsubscribe();
			}
		}
	}

	void allModeSrvCallback(const sensor_msgs::ImageConstPtr& left_camera_data,
			const sensor_msgs::ImageConstPtr& right_camera_data,
			const sensor_msgs::ImageConstPtr& tof_camera_grey_data)
	{
		ROS_INFO("[all_camera_viewer] allModeSrvCallback");
		boost::mutex::scoped_lock lock(m_ServiceMutex);
		// Convert ROS image messages to openCV IplImages
		try
		{
			right_color_image_8U3_ = cv_bridge_0_.imgMsgToCv(right_camera_data, "passthrough");
			left_color_image_8U3_ = cv_bridge_1_.imgMsgToCv(left_camera_data, "passthrough");
			grey_image_32F1_ = cv_bridge_2_.imgMsgToCv(tof_camera_grey_data, "passthrough");

			cv::Mat tmp = right_color_image_8U3_;
			right_color_mat_8U3_ = tmp.clone();

			tmp = left_color_image_8U3_;
			left_color_mat_8U3_ = tmp.clone();
			
			tmp = grey_image_32F1_;
			grey_mat_32F1_ = tmp.clone();
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("[all_camera_viewer] Could not convert stereo images with cv_bridge.");
		}

		// Modifies <code>grey_mat_8U1_</code> and <code>vec_grey_mat_32F1_</code>
		// using <code>grey_mat_32F1_</code>
		updateTofGreyscaleData();

		cv::imshow("TOF grey data", grey_mat_8U1_);

		cv::Mat right_color_8U3;
		cv::resize(right_color_mat_8U3_, right_color_8U3, cv::Size(), 0.5, 0.5);
		cv::imshow("Right color data", right_color_8U3);
		
		cv::Mat left_color_8U3;
		cv::resize(left_color_mat_8U3_, left_color_8U3, cv::Size(), 0.5, 0.5);
		cv::imshow("Left color data", left_color_8U3);
		cv::waitKey(1000);

		ROS_INFO("[all_camera_viewer] allModeSrvCallback [OK]");
	}

	/// Callback is executed, when shared mode is selected.
	/// Left and right is expressed when facing the back of the camera in horitontal orientation.
	void sharedModeSrvCallback(const sensor_msgs::ImageConstPtr& right_camera_data,
			const sensor_msgs::ImageConstPtr& tof_camera_grey_data)
	{
		boost::mutex::scoped_lock lock(m_ServiceMutex);
		ROS_INFO("[all_camera_viewer] sharedModeSrvCallback");
		// Convert ROS image messages to openCV IplImages
		try
		{
			right_color_image_8U3_ = cv_bridge_0_.imgMsgToCv(right_camera_data, "passthrough");
			grey_image_32F1_ = cv_bridge_2_.imgMsgToCv(tof_camera_grey_data, "passthrough");

			cv::Mat tmp = right_color_image_8U3_;
			right_color_mat_8U3_ = tmp.clone();

			tmp = grey_image_32F1_;
			grey_mat_32F1_ = tmp.clone();
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("[all_camera_viewer] Could not convert images with cv_bridge.");
		}

		// Modifies <code>grey_mat_8U1_</code> and <code>vec_grey_mat_32F1_</code>
		// using <code>grey_mat_32F1_</code>
		updateTofGreyscaleData();

		cv::imshow("TOF grey data", grey_mat_8U1_);

		cv::Mat right_color_8U3;
		cv::resize(right_color_mat_8U3_, right_color_8U3, cv::Size(), 0.5, 0.5);
		cv::imshow("Right color data", right_color_8U3);
		cv::waitKey(1000);
	}

	/// Accumulates tof greyscale images, computes the average image out of it
	/// applies histogram normalization and final performs upscaling to generate
	/// better images for calibration
	void updateTofGreyscaleData()
	{
		cv::Mat filtered_grey_mat_32F1 = cv::Mat::zeros(grey_mat_32F1_.rows, grey_mat_32F1_.cols, CV_32FC1);

		// Accumulate greyscale images to remove noise
		if (vec_grey_mat_32F1_.size() <=  tof_image_counter_)
			vec_grey_mat_32F1_.push_back(grey_mat_32F1_);
		else
			vec_grey_mat_32F1_[tof_image_counter_] = grey_mat_32F1_;
		// Update counter
		tof_image_counter_ = (++tof_image_counter_)%5;

		// Compute average out of accumulated tof greyscale data
		for (int i=0; i<vec_grey_mat_32F1_.size(); i++)
			filtered_grey_mat_32F1 += vec_grey_mat_32F1_[i];
		filtered_grey_mat_32F1 * (1/vec_grey_mat_32F1_.size());

		// Update 8bit image
		cv::Mat temp0_grey_mat_8U1;
		cv::Mat temp1_grey_mat_8U1;

		// Convert CV_32FC1 image to CV_8UC1
		double m_scaleMin;
		double m_scaleMax;
		cv::minMaxLoc(filtered_grey_mat_32F1, &m_scaleMin, &m_scaleMax);
		cv::Mat minmat(filtered_grey_mat_32F1.size(), CV_32FC1, cv::Scalar(m_scaleMin));
		filtered_grey_mat_32F1 = cv::Mat(filtered_grey_mat_32F1 - minmat)/ m_scaleMax;
		filtered_grey_mat_32F1.convertTo(temp0_grey_mat_8U1, CV_8U, 256);

		// Perform histogram equalization on 8bit image
		cv::equalizeHist(temp0_grey_mat_8U1, temp1_grey_mat_8U1);

		// Upsample image by a factor of 2 using bilinear interpolation
		// and save result to member variable
		resize(temp1_grey_mat_8U1, grey_mat_8U1_, cv::Size(), 2, 2, cv::INTER_LINEAR);
	}

	/// Callback is executed, when stereo mode is selected
	/// Left and right is expressed when facing the back of the camera in horizontal orientation.
	void stereoModeSrvCallback(const sensor_msgs::ImageConstPtr& left_camera_data,
			const sensor_msgs::ImageConstPtr& right_camera_data)
	{
		ROS_INFO("[all_camera_viewer] stereoModeSrvCallback");
		boost::mutex::scoped_lock lock(m_ServiceMutex);
		// Convert ROS image messages to openCV IplImages
		try
		{
			right_color_image_8U3_ = cv_bridge_0_.imgMsgToCv(right_camera_data, "passthrough");
			left_color_image_8U3_ = cv_bridge_1_.imgMsgToCv(left_camera_data, "passthrough");

			cv::Mat tmp = right_color_image_8U3_;
			right_color_mat_8U3_ = tmp.clone();

			tmp = left_color_image_8U3_;
			left_color_mat_8U3_ = tmp.clone();
		}
		catch (sensor_msgs::CvBridgeException& e)
		{
			ROS_ERROR("[all_camera_viewer] Could not convert stereo images with cv_bridge.");
		}
		
		cv::Mat right_color_8U3;
		cv::resize(right_color_mat_8U3_, right_color_8U3, cv::Size(), 0.5, 0.5);
		cv::imshow("Right color data", right_color_8U3);
		
		cv::Mat left_color_8U3;
		cv::resize(left_color_mat_8U3_, left_color_8U3, cv::Size(), 0.5, 0.5);
		cv::imshow("Left color data", left_color_8U3);
		cv::waitKey(1000);

		ROS_INFO("[all_camera_viewer] stereoModeSrvCallback [OK]");
	}

	bool saveCameraImagesServiceCallback(cob_srvs::AcquireCalibrationImages::Request &req,
			cob_srvs::AcquireCalibrationImages::Response &res)
	{
		ROS_INFO("[all_camera_viewer] Service Callback");
		boost::mutex::scoped_lock lock(m_ServiceMutex);

		if (req.reset_image_counter) image_counter_ = 0;

		char counterBuffer [50];
		sprintf(counterBuffer, "%04d", image_counter_);

		if (use_right_color_camera_ && right_color_mat_8U3_.empty())
		{
			ROS_INFO("[all_camera_viewer] Right color image not available");
			return false;
		}
		else
		{
			std::stringstream ss;
			ss << "right_color_image_";
			ss << counterBuffer;
			ss << ".bmp";
			cv::imwrite(absolute_output_directory_path_ + ss.str(), right_color_mat_8U3_);
			ROS_INFO("[all_camera_viewer] Saved right color image %d to %s", image_counter_, ss.str().c_str());
		}
			
		if (use_left_color_camera_ && left_color_mat_8U3_.empty())
		{
			ROS_INFO("[all_camera_viewer] Left color image not available");
			return false;
		}
		else
		{
			std::stringstream ss;
			ss << "left_color_image_";
			ss << counterBuffer;
			ss << ".bmp";
			cv::imwrite(absolute_output_directory_path_ + ss.str(), left_color_mat_8U3_);
			ROS_INFO("[all_camera_viewer] Saved left color image %d to %s", image_counter_, ss.str().c_str());
		}

		
		if (use_tof_camera_ && grey_mat_8U1_.empty())
		{
			ROS_INFO("[all_camera_viewer] Tof grayscale image not available");
			return false;
		}
		else
		{
			std::stringstream ss;
			ss << "tof_grey_image_";
			ss << counterBuffer;
			ss << ".bmp";
			cv::imwrite(absolute_output_directory_path_ + ss.str(), grey_mat_8U1_);
			ROS_INFO("[all_camera_viewer] Saved tof gray image %d to %s", image_counter_, ss.str().c_str());
		}

		image_counter_++;

		return true;
	}

	unsigned long loadParameters()
	{
		std::string tmp_string;

		/// Parameters are set within the launch file
		if (node_handle_.getParam("all_camera_viewer/use_tof_camera", use_tof_camera_) == false)
		{
			ROS_ERROR("[all_camera_viewer] 'use_tof_camera' not specified");
			return false;
		}
		ROS_INFO("use tof camera: %d", use_tof_camera_);

		if (node_handle_.getParam("all_camera_viewer/use_right_color_camera", use_right_color_camera_) == false)
		{
			ROS_ERROR("[all_camera_viewer] 'use_right_color_camera' not specified");
			return false;
		}
		ROS_INFO("use right color camera: %d", use_right_color_camera_);

		if (node_handle_.getParam("all_camera_viewer/use_left_color_camera", use_left_color_camera_) == false)
		{
			ROS_ERROR("[all_camera_viewer] 'use_left_color_camera' not specified");
			return false;
		}
		ROS_INFO("use left color camera: %d", use_left_color_camera_);

		if (node_handle_.getParam("all_camera_viewer/absolute_output_directory_path", absolute_output_directory_path_) == false)
		{
			ROS_ERROR("[all_camera_viewer] 'absolute_output_directory_path' not specified");
			return false;
		}
		ROS_INFO("use left color camera: %d", use_left_color_camera_);

		return true;
	}

};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, spezify name of node
	ros::init(argc, argv, "all_camera_viewer");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;
	
	/// Create camera node class instance	
	AllCameraViewer camera_node(nh);

	/// Initialize camera node
	if (!camera_node.init()) return 0;

	ros::spin();

	return 0;
}
