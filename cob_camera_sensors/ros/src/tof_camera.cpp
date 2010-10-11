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
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <cob_srvs/GetTOFImages.h>

// external includes
#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <cob_vision_utils/CameraSensorToolbox.h>
#include <cob_vision_utils/GlobalDefines.h>
#include <cob_vision_utils/VisionUtils.h>

#include <boost/thread/mutex.hpp>

using namespace ipa_CameraSensors;

class CobTofCameraNode
{
	enum t_Mode
	{
		MODE_TOPIC = 0,
		MODE_SERVICE
	};

private:
	ros::NodeHandle node_handle_;	///< Node handle

	image_transport::ImageTransport image_transport_;	///< Image transport instance
	image_transport::CameraPublisher xyz_image_publisher_;	///< Publishes xyz image data
	image_transport::CameraPublisher grey_image_publisher_;	///< Publishes grey image data
	ros::Publisher topicPub_pointCloud_;
	ros::Publisher topicPub_pointCloud2_;

	sensor_msgs::CameraInfo camera_info_msg_;    ///< ROS camera information message (e.g. holding intrinsic parameters)

	ros::ServiceServer camera_info_service_;		///< Service to set/modify camera parameters
	ros::ServiceServer image_service_;

	AbstractRangeImagingSensorPtr tof_camera_;     ///< Time-of-flight camera instance
	
	std::string config_directory_; ///< Directory of related IPA configuration file
	int tof_camera_index_;	///< Camera index of the color camera for IPA configuration file
	ipa_CameraSensors::t_cameraType tof_camera_type_; ///< Type of tof camera
	bool filter_xyz_by_amplitude_;
	bool filter_xyz_tearoff_edges_;
	int lower_amplitude_threshold_;
	int upper_amplitude_threshold_;
	double tearoff_tear_half_fraction_;

	cv::Mat xyz_image_32F3_;	/// OpenCV image holding the point cloud
	cv::Mat grey_image_32F1_;	/// OpenCV image holding the amplitude values

	CobTofCameraNode::t_Mode ros_node_mode_;	///< Specifies if node is started as topic or service
	boost::mutex service_mutex_;

	bool publish_point_cloud_;
	bool publish_point_cloud_2_;

public:
	/// Constructor.
    CobTofCameraNode(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle),
	  image_transport_(node_handle),
      tof_camera_(AbstractRangeImagingSensorPtr()),
      xyz_image_32F3_(cv::Mat()),
      grey_image_32F1_(cv::Mat()),
      publish_point_cloud_(false),
      publish_point_cloud_2_(false)
    {
            /// Void
    }
	
	/// Destructor
	~CobTofCameraNode()
    {
	tof_camera_->Close();
    }

    /// Initializes and opens the time-of-flight camera sensor.
	/// @return <code>false</code> on failure, <code>true</code> otherwise
    bool init()
    {
		if (loadParameters() == false)
		{
			ROS_ERROR("[color_camera] Could not read all parameters from launch file");
			return false;
		}		
		

		if (tof_camera_->Init(config_directory_, tof_camera_index_) & ipa_CameraSensors::RET_FAILED)
		{

			std::stringstream ss;
			ss << "Initialization of tof camera ";
			ss << tof_camera_index_;
			ss << " failed";
			ROS_ERROR("[tof_camera] %s", ss.str().c_str());
			tof_camera_ = AbstractRangeImagingSensorPtr();
			return false;
		}
	
		if (tof_camera_->Open() & ipa_CameraSensors::RET_FAILED)
		{
			std::stringstream ss;
			ss << "Could not open tof camera ";
			ss << tof_camera_index_;
			ROS_ERROR("[tof_camera] %s", ss.str().c_str());
			tof_camera_ = AbstractRangeImagingSensorPtr();
			return false;
		}

		/// Read camera properties of range tof sensor
		ipa_CameraSensors::t_cameraProperty cameraProperty;
		cameraProperty.propertyID = ipa_CameraSensors::PROP_CAMERA_RESOLUTION;
		tof_camera_->GetProperty(&cameraProperty);
		int range_sensor_width = cameraProperty.cameraResolution.xResolution;
		int range_sensor_height = cameraProperty.cameraResolution.yResolution;
		cv::Size range_image_size(range_sensor_width, range_sensor_height);

		/// Setup camera toolbox
		ipa_CameraSensors::CameraSensorToolboxPtr tof_sensor_toolbox = ipa_CameraSensors::CreateCameraSensorToolbox();
		tof_sensor_toolbox->Init(config_directory_, tof_camera_->GetCameraType(), tof_camera_index_, range_image_size);

		cv::Mat intrinsic_mat = tof_sensor_toolbox->GetIntrinsicMatrix(tof_camera_type_, tof_camera_index_);
		cv::Mat distortion_map_X = tof_sensor_toolbox->GetDistortionMapX(tof_camera_type_, tof_camera_index_);
		cv::Mat distortion_map_Y = tof_sensor_toolbox->GetDistortionMapY(tof_camera_type_, tof_camera_index_);
		tof_camera_->SetIntrinsics(intrinsic_mat, distortion_map_X, distortion_map_Y);

	        /// Advertise service for other nodes to set intrinsic calibration parameters
		camera_info_service_ = node_handle_.advertiseService("set_camera_info", &CobTofCameraNode::setCameraInfo, this);
		image_service_ = node_handle_.advertiseService("get_images", &CobTofCameraNode::imageSrvCallback, this);
		xyz_image_publisher_ = image_transport_.advertiseCamera("image_xyz", 1);
		grey_image_publisher_ = image_transport_.advertiseCamera("image_grey", 1);
		if(publish_point_cloud_2_) topicPub_pointCloud2_ = node_handle_.advertise<sensor_msgs::PointCloud2>("point_cloud2", 1);
		if(publish_point_cloud_) topicPub_pointCloud_ = node_handle_.advertise<sensor_msgs::PointCloud>("point_cloud", 1);

		cv::Mat d = tof_sensor_toolbox->GetDistortionParameters(tof_camera_type_, tof_camera_index_);
		camera_info_msg_.D[0] = d.at<double>(0, 0);
		camera_info_msg_.D[1] = d.at<double>(0, 1);
		camera_info_msg_.D[2] = d.at<double>(0, 2);
		camera_info_msg_.D[3] = d.at<double>(0, 3);
		camera_info_msg_.D[4] = 0;
	
		cv::Mat k = tof_sensor_toolbox->GetIntrinsicMatrix(tof_camera_type_, tof_camera_index_);
		camera_info_msg_.K[0] = k.at<double>(0, 0);
		camera_info_msg_.K[1] = k.at<double>(0, 1);
		camera_info_msg_.K[2] = k.at<double>(0, 2);
		camera_info_msg_.K[3] = k.at<double>(1, 0);
		camera_info_msg_.K[4] = k.at<double>(1, 1);
		camera_info_msg_.K[5] = k.at<double>(1, 2);
		camera_info_msg_.K[6] = k.at<double>(2, 0);
		camera_info_msg_.K[7] = k.at<double>(2, 1);
		camera_info_msg_.K[8] = k.at<double>(2, 2);

		camera_info_msg_.width = range_sensor_width;		
		camera_info_msg_.height = range_sensor_height;

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

    	/// Continuously advertises xyz and grey images.
	bool spin()
	{
		boost::mutex::scoped_lock lock(service_mutex_);
		sensor_msgs::Image::Ptr xyz_image_msg_ptr;
		sensor_msgs::Image::Ptr grey_image_msg_ptr;
		sensor_msgs::CameraInfo tof_image_info;
	
		if(tof_camera_->AcquireImages(0, &grey_image_32F1_, &xyz_image_32F3_, false, false, ipa_CameraSensors::INTENSITY) & ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("[tof_camera] Tof image acquisition failed");
			return false;
		}

		/// Filter images by amplitude and remove tear-off edges
		//if(filter_xyz_tearoff_edges_ || filter_xyz_by_amplitude_)
		//	ROS_ERROR("[tof_camera] FUNCTION UNCOMMENT BY JSF");
		if(filter_xyz_tearoff_edges_) ipa_Utils::FilterTearOffEdges(xyz_image_32F3_, 0, (float)tearoff_tear_half_fraction_);
		if(filter_xyz_by_amplitude_) ipa_Utils::FilterByAmplitude(xyz_image_32F3_, grey_image_32F1_, 0, 0, lower_amplitude_threshold_, upper_amplitude_threshold_);

		try
		{
			IplImage img = xyz_image_32F3_;
			xyz_image_msg_ptr = sensor_msgs::CvBridge::cvToImgMsg(&img, "passthrough");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("[tof_camera] Could not convert 32bit xyz IplImage to ROS message");
			return false;
		}

		try
		{
			IplImage img = grey_image_32F1_;
			grey_image_msg_ptr = sensor_msgs::CvBridge::cvToImgMsg(&img, "passthrough");
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("[tof_camera] Could not convert 32bit grey IplImage to ROS message");
			return false;
		}

		/// Set time stamp
		ros::Time now = ros::Time::now();
		xyz_image_msg_ptr->header.stamp = now;
		xyz_image_msg_ptr->header.frame_id = "head_tof_camera_link";
		grey_image_msg_ptr->header.stamp = now;
		grey_image_msg_ptr->header.frame_id = "head_tof_camera_link";

		tof_image_info = camera_info_msg_;
		tof_image_info.width = grey_image_32F1_.cols;
		tof_image_info.height = grey_image_32F1_.rows;
		tof_image_info.header.stamp = now;
		tof_image_info.header.frame_id = "head_tof_camera_link";

		/// publish message
		xyz_image_publisher_.publish(*xyz_image_msg_ptr, tof_image_info);
		grey_image_publisher_.publish(*grey_image_msg_ptr, tof_image_info);

		if(publish_point_cloud_) publishPointCloud(now);
		if(publish_point_cloud_2_) publishPointCloud2(now);

		return true;
	}

    void publishPointCloud(ros::Time now)
    {
        ROS_DEBUG("convert xyz_image to point_cloud");
        sensor_msgs::PointCloud pc_msg;
		// create point_cloud message
		pc_msg.header.stamp = now;
		pc_msg.header.frame_id = "head_tof_camera_link";

		cv::Mat cpp_xyz_image_32F3 = xyz_image_32F3_;
		cv::Mat cpp_grey_image_32F1 = grey_image_32F1_;

		float* f_ptr = 0;
		for (int row = 0; row < cpp_xyz_image_32F3.rows; row++)
		{
			f_ptr = cpp_xyz_image_32F3.ptr<float>(row);
			for (int col = 0; col < cpp_xyz_image_32F3.cols; col++)
			{
				geometry_msgs::Point32 pt;
				pt.x = f_ptr[3*col + 0];
				pt.y = f_ptr[3*col + 1];
				pt.z = f_ptr[3*col + 2];
				pc_msg.points.push_back(pt);
			}
		}
        topicPub_pointCloud_.publish(pc_msg);
    }

	void publishPointCloud2(ros::Time now)
	{
		cv::Mat cpp_xyz_image_32F3 = xyz_image_32F3_;
		cv::Mat cpp_confidence_mask_32F1 = grey_image_32F1_;

		sensor_msgs::PointCloud2 pc_msg;
		// create point_cloud message
		pc_msg.header.stamp = now;
		pc_msg.header.frame_id = "head_tof_camera_link";
		pc_msg.width = cpp_xyz_image_32F3.cols;
		pc_msg.height = cpp_xyz_image_32F3.rows;
		pc_msg.fields.resize(4);
		pc_msg.fields[0].name = "x";
		pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
		pc_msg.fields[1].name = "y";
		pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
		pc_msg.fields[2].name = "z";
		pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
		//pc_msg.fields[3].name = "rgb";
		//pc_msg.fields[3].datatype = sensor_msgs::PointField::UINT32;
		pc_msg.fields[4].name = "confidence";
		pc_msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
		//pc_msg.fields[5].name = "features";
		//pc_msg.fields[5].datatype = sensor_msgs::PointField::UINT8;
		int offset = 0;
		for (size_t d = 0; d < pc_msg.fields.size(); ++d, offset += 4)
		{
			pc_msg.fields[d].offset = offset;
		}
		pc_msg.point_step = offset;
		pc_msg.row_step = pc_msg.point_step * pc_msg.width;
		pc_msg.data.resize (pc_msg.width*pc_msg.height*pc_msg.point_step);
		pc_msg.is_dense = true;
		pc_msg.is_bigendian = false;

		float* f_ptr = 0;
		//unsigned char c_dummy[3] = {0,0,0};
		float* g_ptr = 0;
		//unsigned char* ft_dummy = 0;
		int pc_msg_idx=0;
		for (int row = 0; row < cpp_xyz_image_32F3.rows; row++)
		{
			f_ptr = cpp_xyz_image_32F3.ptr<float>(row);
			g_ptr = cpp_confidence_mask_32F1.ptr<float>(row);
			for (int col = 0; col < cpp_xyz_image_32F3.cols; col++, pc_msg_idx++)
			{
				memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step], &f_ptr[3*col], 3*sizeof(float));
				//memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step + pc_msg.fields[3].offset], &c_dummy, 3*sizeof(unsigned char));
				memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step + pc_msg.fields[3].offset], &g_ptr[col], sizeof(float));
				//memcpy(&pc_msg.data[pc_msg_idx * pc_msg.point_step + pc_msg.fields[5].offset], &ft_dummy, sizeof(unsigned char));
			}
		}
		topicPub_pointCloud2_.publish(pc_msg);
	}

	bool imageSrvCallback(cob_srvs::GetTOFImages::Request &req,
			cob_srvs::GetTOFImages::Response &res)
	{
		boost::mutex::scoped_lock lock(service_mutex_);
		// Convert openCV IplImages to ROS messages
		try
		{
			IplImage grey_img = grey_image_32F1_;
			IplImage xyz_img = xyz_image_32F3_;
			res.greyImage = *(sensor_msgs::CvBridge::cvToImgMsg(&grey_img, "passthrough"));
			res.xyzImage = *(sensor_msgs::CvBridge::cvToImgMsg(&xyz_img, "passthrough"));
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("[tof_camera_type_node] Could not convert IplImage to ROS message");
		}

		// Set time stamp
		ros::Time now = ros::Time::now();
		res.greyImage.header.stamp = now;
		res.xyzImage.header.stamp = now;
		return true;
	}

	bool loadParameters()
	{
		std::string tmp_string = "NULL";
               
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/configuration_files", config_directory_) == false)
		{
				ROS_ERROR("[tof_camera] Path to xml configuration for tof camera not specified");
				return false;
		}

		ROS_INFO("Configuration directory: %s", config_directory_.c_str());

		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/tof_camera_index", tof_camera_index_) == false)
		{
			ROS_ERROR("[tof_camera] 'tof_camera_index' (0 or 1) not specified");
			return false;
		}

		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/tof_camera_type", tmp_string) == false)
		{
			ROS_ERROR("[tof_camera] 'tof_camera_type' not specified");
			return false;
		}
		if (tmp_string == "CAM_SWISSRANGER") 
		{
			tof_camera_ = ipa_CameraSensors::CreateRangeImagingSensor_Swissranger();
			tof_camera_type_ = ipa_CameraSensors::CAM_SWISSRANGER;
		}
		else if (tmp_string == "CAM_VIRTUAL") 
		{
			tof_camera_ = ipa_CameraSensors::CreateRangeImagingSensor_VirtualCam();
			tof_camera_type_ = ipa_CameraSensors::CAM_VIRTUALRANGE;
		}
		else
		{
			std::string str = "[tof_camera] Camera type '" + tmp_string + "' unknown, try 'CAM_SWISSRANGER'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}

		ROS_INFO("Camera type: %s_%d", tmp_string.c_str(), tof_camera_index_);

		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/filter_xyz_by_amplitude", filter_xyz_by_amplitude_) == false)
		{
			ROS_ERROR("[tof_camera] 'filter_xyz_by_amplitude not specified");
			return false;
		}
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/filter_xyz_tearoff_edges", filter_xyz_tearoff_edges_) == false)
		{
			ROS_ERROR("[tof_camera] 'filter_xyz_tearoff_edges_' not specified");
			return false;
		}
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/lower_amplitude_threshold", lower_amplitude_threshold_) == false)
		{
			ROS_ERROR("[tof_camera] 'lower_amplitude_threshold' not specified");
			return false;
		}
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/upper_amplitude_threshold", upper_amplitude_threshold_) == false)
		{
			ROS_ERROR("[tof_camera] 'upper_amplitude_threshold' not specified");
			return false;
		}
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/tearoff_pi_half_fraction", tearoff_tear_half_fraction_) == false)
		{
			ROS_ERROR("[tof_camera] 'tearoff_pi_half_fraction' not specified");
			return false;
		}
		/// Parameters are set within the launch file
		if (node_handle_.getParam("tof_camera/ros_node_mode", tmp_string) == false)
		{
			ROS_ERROR("[tof_camera] Mode for tof camera node not specified");
			return false;
		}
		if (tmp_string == "MODE_SERVICE")
		{
			ros_node_mode_ = CobTofCameraNode::MODE_SERVICE;
		}
		else if (tmp_string == "MODE_TOPIC")
		{
			ros_node_mode_ = CobTofCameraNode::MODE_TOPIC;
		}
		else
		{
			std::string str = "[tof_camera] Mode '" + tmp_string + "' unknown, try 'MODE_SERVICE' or 'MODE_TOPIC'";
			ROS_ERROR("%s", str.c_str());
			return false;
		}

		if(node_handle_.getParam("tof_camera/publish_point_cloud", publish_point_cloud_) == false)
		{
			ROS_WARN("[tof_camera] Flag for publishing PointCloud not set, falling back to default (false)");
		}
		if(node_handle_.getParam("tof_camera/publish_point_cloud_2", publish_point_cloud_2_) == false)
		{
			ROS_WARN("[tof_camera] Flag for publishing PointCloud2 not set, falling back to default (false)");
		}


		ROS_INFO("ROS node mode: %s", tmp_string.c_str());

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

    //ros::spin();
	ros::Rate rate(10);
	while(nh.ok())
	{
		camera_node.spin();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
