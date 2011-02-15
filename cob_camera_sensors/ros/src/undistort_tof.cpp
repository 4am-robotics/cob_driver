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
 * Author: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * Date of creation: Dec 2010
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

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

// external includes
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace message_filters;

typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> SyncPolicy;


class UndistortTOF
{
private:
	ros::NodeHandle node_handle_;
	sensor_msgs::PointCloud2 pc2_msg_;

	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc2_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
	message_filters::Synchronizer<SyncPolicy> sub_sync_;
	ros::Publisher pub_pc2_;

public:
	UndistortTOF(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  sub_sync_(SyncPolicy(3))
	  {
		sub_sync_.connectInput(sub_pc2_, sub_camera_info_);
		sub_sync_.registerCallback(boost::bind(&UndistortTOF::Undistort, this, _1, _2));
		sub_pc2_.subscribe(node_handle_, "tof/point_cloud2", 1);
		sub_camera_info_.subscribe(node_handle_, "tof/camera_info", 1);
		pub_pc2_ = node_handle_.advertise<sensor_msgs::PointCloud2>("point_cloud_undistorted", 1);
	  }

	// Return value is in m
	unsigned long GetCalibratedZSwissranger(int u, int v, int width, float& zCalibrated)
	{
		//zCalibrated = (float) m_Z[v*width + u];

		//return RET_OK;
	}

	// u and v are assumed to be distorted coordinates
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y)
	{
		// Use intrinsic camera parameters
		/*double fx, fy, cx, cy;

		fx = m_intrinsicMatrix.at<double>(0, 0);
		fy = m_intrinsicMatrix.at<double>(1, 1);

		cx = m_intrinsicMatrix.at<double>(0, 2);
		cy = m_intrinsicMatrix.at<double>(1, 2);

		// Fundamental equation: u = (fx*x)/z + cx
		if (fx == 0)
		{
			std::cerr << "ERROR - Swissranger::GetCalibratedXYZ:" << std::endl;
			std::cerr << "\t ... fx is 0.\n";
			return RET_FAILED;
		}
		x = (float) (z*(u-cx)/fx) ;

		// Fundamental equation: v = (fy*y)/z + cy
		if (fy == 0)
		{
			std::cerr << "ERROR - Swissranger::GetCalibratedXYZ:" << std::endl;
			std::cerr << "\t ... fy is 0.\n";
			return RET_FAILED;
		}
		y = (float) (z*(v-cy)/fy);

		return RET_OK;*/
	}

	//void Undistort(const sensor_msgs::PointCloud2ConstPtr& tof_camera_data, const sensor_msgs::CameraInfoConstPtr& camera_info)
	void Undistort(const boost::shared_ptr<sensor_msgs::PointCloud2 const>& tof_camera_data, const sensor_msgs::CameraInfoConstPtr& camera_info)
	{
		cv::Mat D = cv::Mat(1,4,CV_64FC1);
		D.at<double>(0,0) = camera_info->D[0];
		D.at<double>(0,1) = camera_info->D[1];
		D.at<double>(0,2) = camera_info->D[2];
		D.at<double>(0,3) = camera_info->D[3];
		cv::Mat cam_matrix = cv::Mat::zeros(3,3,CV_64FC1);
		cam_matrix.at<double>(0,0) = camera_info->K[0];
		cam_matrix.at<double>(0,2) = camera_info->K[2];
		cam_matrix.at<double>(1,1) = camera_info->K[4];
		cam_matrix.at<double>(1,2) = camera_info->K[5];
		cam_matrix.at<double>(2,2) = 1;

		int z_offset = 0, i_offset = 0, c_offset = 0, x_offset = 0, y_offset = 0;;
		for (size_t d = 0; d < tof_camera_data->fields.size(); ++d)
		{
			if(tof_camera_data->fields[d].name == "x")
				x_offset = tof_camera_data->fields[d].offset;
			if(tof_camera_data->fields[d].name == "y")
				y_offset = tof_camera_data->fields[d].offset;
			if(tof_camera_data->fields[d].name == "z")
				z_offset = tof_camera_data->fields[d].offset;
			if(tof_camera_data->fields[d].name == "intensity")
				i_offset = tof_camera_data->fields[d].offset;
			if(tof_camera_data->fields[d].name == "confidence")
				c_offset = tof_camera_data->fields[d].offset;
		}
		cv::Mat z_image = cv::Mat(tof_camera_data->height, tof_camera_data->width,CV_32FC1);
		cv::Mat intensity_image = cv::Mat(tof_camera_data->height, tof_camera_data->width,CV_32FC1);
		float* f_ptr = 0;
		float* i_ptr = 0;
		int pc_msg_idx=0;
		for (int row = 0; row < z_image.rows; row++)
		{
			f_ptr = z_image.ptr<float>(row);
			i_ptr = intensity_image.ptr<float>(row);
			for (int col = 0; col < z_image.cols; col++, pc_msg_idx++)
			{
				memcpy(&f_ptr[col], &tof_camera_data->data[pc_msg_idx * tof_camera_data->point_step + z_offset], sizeof(float));
				memcpy(&i_ptr[col], &tof_camera_data->data[pc_msg_idx * tof_camera_data->point_step + i_offset], sizeof(float));
			}
		}

		cv::imshow("distorted", z_image);
		// Undistort
		cv::Mat z_image_undistorted;
		cv::undistort(z_image, z_image_undistorted, cam_matrix, D);
		cv::Mat intensity_image_undistorted;
		cv::undistort(intensity_image, intensity_image_undistorted, cam_matrix, D);

		cv::imshow("undistorted", z_image_undistorted);
		cv::waitKey(20);

		sensor_msgs::PointCloud2 pc_pub = *(tof_camera_data.get());
		// Calculate X and Y based on instrinsic rotation and translation
		double fx, fy, cx, cy;

		fx = cam_matrix.at<double>(0,0);
		fy = cam_matrix.at<double>(1, 1);

		cx = cam_matrix.at<double>(0, 2);
		cy = cam_matrix.at<double>(1, 2);


		for (size_t d = 0; d < pc_pub.fields.size(); ++d)
		{
		}
		for(unsigned int row=0; row<pc_pub.height; row++)
		{
			float* z =  z_image_undistorted.ptr<float>(row);
			//f_ptr = (float*)(cartesianImageData + row*widthStepCartesianImage);

			for (unsigned int col=0; col<pc_pub.width; col++)
			{
				memcpy(&pc_pub.data[row * col * pc_pub.point_step + z_offset], &z[col], sizeof(float));
				float x = (float) (z[col]*(col-cx)/fx);
				float y =  (float) (z[col]*(row-cy)/fy);
				memcpy(&pc_pub.data[row * col * pc_pub.point_step + x_offset], &x, sizeof(float));
				memcpy(&pc_pub.data[row * col * pc_pub.point_step + y_offset], &y, sizeof(float));
			}
		}
		pub_pc2_.publish(pc_pub);
	}

};


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, spezify name of node
	ros::init(argc, argv, "undistort_tof");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/// Create camera node class instance
	UndistortTOF undistort_tof(nh);

	/// Initialize camera node
	//if (!camera_node.init()) return 0;

	ros::spin();

	return 0;
}
