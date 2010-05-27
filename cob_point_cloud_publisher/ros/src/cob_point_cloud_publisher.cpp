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
 * ROS package name: cob_point_cloud_publisher
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
 *
 * Date of creation: May 2010
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

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

// ROS service includes
//--

// external includes
//--

//####################
//#### PcPublisher class ####
class PcPublisher
{
	private:
		IplImage* xyz_image_32F3_;	///< Received point cloud form tof sensor
		sensor_msgs::CvBridge cv_bridge_; ///< Converts ROS image messages to openCV IplImages
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n_;
                
        // topics to publish
        ros::Publisher topicPub_pointCloud_;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_xyzImage_;
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        //--

        // Constructor
        PcPublisher()
        {
            topicPub_pointCloud_ = n_.advertise<sensor_msgs::PointCloud>("point_cloud", 1);
            topicSub_xyzImage_ = n_.subscribe("xyz_image", 1, &PcPublisher::topicCallback_xyzImage, this);
        }
        
        // Destructor
        ~PcPublisher() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_xyzImage(const sensor_msgs::Image::ConstPtr& tof_camera_xyz_data)
        {
            ROS_INFO("convert xyz_image to point_cloud");
            sensor_msgs::PointCloud pc_msg;
			// create point_cloud message
			xyz_image_32F3_ = cv_bridge_.imgMsgToCv(tof_camera_xyz_data, "passthrough");
			float* f_ptr = 0;
			for (int row = 0; row < xyz_image_32F3_->height; row++)
			{
				f_ptr = (float*)(xyz_image_32F3_->imageData + row*xyz_image_32F3_->widthStep);
				for (int col = 0; col < xyz_image_32F3_->height; col++)
				{
					geometry_msgs::Point32 pt;
					pt.x = f_ptr[3*col + 0];
					pt.x = f_ptr[3*col + 1];
					pt.x = f_ptr[3*col + 2];
					pc_msg.points.push_back(pt); 
				}
			}
			
			pc_msg.header.stamp = ros::Time::now();
			
			//TODO: fill message with xyz_msg values
            
            topicPub_pointCloud_.publish(pc_msg);
        }
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob_point_cloud_publisher");
    
    PcPublisher pcPublisher;
    
    ros::spin();

    return 0;
}
