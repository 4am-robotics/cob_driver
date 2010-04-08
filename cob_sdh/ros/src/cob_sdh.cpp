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
 * ROS package name: sdh
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
 * Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
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

// ROS message includes
#include <cob_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
//// #include <cob_msgs/TactileMatrix.h>

// ROS service includes
#include <cob_srvs/Init.h>

// external includes

#include <cob_sdh/sdh.h>
#include <cob_sdh/dsa.h>
#include <cob_sdh/util.h>
#include <cob_sdh/sdhlibrary_settings.h>
#include <cob_sdh/basisdef.h>

//USING_NAMESPACE_SDH


//####################
//#### node class ####
class NodeClass
{
	//
	public:
		// create a handle for this node, initialize node
		ros::NodeHandle n;

		// declaration of topics to publish
		ros::Publisher topicPub_JointState;
		ros::Publisher topicPub_ActuatorState;
		////ros::Publisher topicPub_TactileMatrix;

		// declaration of topics to subscribe, callback is called for new messages arriving
		ros::Subscriber topicSub_JointCommand;

		// service servers
		ros::ServiceServer srvServer_Init;

		// service clients
		//--

		// global variables

		SDH::cSDH *sdh;
		SDH::cDSA *dsa;  

		std::string sdhdevicetype;
		std::string sdhdevicestring;
		int sdhdevicenum;
		std::string dsadevicestring;
		int dsadevicenum;

		bool isInitialized;
		bool isDSAInitialized;
		int DOF;


		// Constructor
		NodeClass()
		{
			// initialize global variables
			isInitialized = false;
			isDSAInitialized = false;
			DOF = 7; // DOFs of sdh

			// implementation of topics to publish
			topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);
			////topicPub_TactileMatrix = n.advertise<cob_msgs::TactileMatrix>("tactile_data", 1);

			// implementation of topics to subscribe

			n.param("sdhdevicetype", sdhdevicetype, std::string("PEAK"));
			//n.param("sdhdevicestring", sdhdevicestring, std::string("/dev/pcan%d"));
			n.param("sdhdevicestring", sdhdevicestring, std::string("/dev/pcan0"));
			n.param("sdhdevicenum", sdhdevicenum, 0);
			//n.param("dsadevicestring", dsadevicestring, std::string("/dev/pcan%d"));
			n.param("dsadevicestring", dsadevicestring, std::string("/dev/ttyS0"));
			n.param("dsadevicenum", dsadevicenum, 0);

			// pointer to sdh
			sdh = new SDH::cSDH(false, false, 2); //(_use_radians=false, bool _use_fahrenheit=false, int _debug_level=0)

			topicSub_JointCommand = n.subscribe("joint_commands", 1, &NodeClass::topicCallback_JointCommand, this);

			// implementation of service servers
			srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
		}

		// Destructor
		~NodeClass() 
		{
			sdh->Close();
			delete sdh;
		}

		// topic callback functions 
		// function will be called when a new message arrives on a topic
		void topicCallback_JointCommand(const cob_msgs::JointCommand::ConstPtr& msg)
		{
			ROS_INFO("Received new JointCommand");

			if(isInitialized == true)
			{
				//TODO: send msg data to hardware
				std::vector<int> axes;
				std::vector<double> axes_angles;
				for(int i=0; i<DOF; i++)
				{
					axes.push_back(i);
					axes_angles.push_back(msg->positions[i]);

				}

				try
				{
					sdh->SetAxisTargetAngle( axes, axes_angles );
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}

				try
				{
					sdh->MoveHand(true);
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			}
		}

		// service callback functions
		// function will be called when a service is querried
		bool srvCallback_Init(cob_srvs::Init::Request &req,
				cob_srvs::Init::Response &res )
		{
			ROS_INFO("Initializing sdh");

			//TODO: read from parameter
			//int _net=0;
			unsigned long _baudrate=1000000;
			double _timeout=0.1;
			unsigned long _id_read=43;
			unsigned long _id_write=42;

			n.getParam("sdhdevicetype", sdhdevicetype);
			n.getParam("sdhdevicestring", sdhdevicestring);
			n.getParam("sdhdevicenum", sdhdevicenum);

			try
			{
				if(sdhdevicetype.compare("RS232")==0)
				{
					sdh->OpenRS232( sdhdevicenum, 115200, 1, sdhdevicestring.c_str());
					ROS_INFO("Initialized RS232 for SDH");
					isInitialized = true;
				}
				if(sdhdevicetype.compare("PEAK")==0)
				{
					ROS_INFO("Starting initializing PEAKCAN");
					sdh->OpenCAN_PEAK(_baudrate, _timeout, _id_read, _id_write, sdhdevicestring.c_str());
					ROS_INFO("Initialized PEAK CAN for SDH");
					isInitialized = true;
				}
				if(sdhdevicetype.compare("ESD")==0)
				{
					ROS_INFO("Starting init ESD");
					sdh->OpenCAN_ESD(0, _baudrate, _timeout, _id_read, _id_write );
					ROS_INFO("Initialized ESDCAN for SDH");
					isInitialized = true;
				}


			}
			catch (SDH::cSDHLibraryException* e)
			{
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			n.getParam("dsadevicestring", dsadevicestring);
			n.getParam("dsadevicenum", dsadevicenum);
			try
			{
				dsa = new SDH::cDSA(0,dsadevicenum, dsadevicestring.c_str());
				dsa->SetFramerate( 1, 1 );
				ROS_INFO("Initialized RS232 for DSA Tactile Sensors");
				isDSAInitialized = true;
			}
			catch (SDH::cSDHLibraryException* e)
			{
				isDSAInitialized = false;
				ROS_ERROR("An exception was caught: %s", e->what());
				delete e;
			}
			return true;
		}




		void updateJointState()
		{
			//ROS_INFO("updateJointState");
			std::vector<double> actualAngles;

			if(isInitialized == true)
			{
				//get actual joint positions 
				std::vector<int> axes;

				for(int i=0; i<DOF; i++)
				{
					axes.push_back(i);
				}

				try
				{
					actualAngles = sdh->GetAxisActualAngle( axes );
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			}
			else
			{
				actualAngles.resize(DOF);
				for(int i=0; i<DOF; i++)
				{
					actualAngles[i] = 0.0;
				}
			}

			// fill message
			// NOTE: hardware has 8 DOFs, but two cuppled ones; joints palm_finger11 and palm_finger21 are actuated synchronously
			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.resize(DOF+1);
			msg.position.resize(DOF+1);

			// set joint names and map them to angles TODO: don't know if assignment is correct
			msg.name[0] = "joint_thumb1_thumb2";
			msg.position[0] = actualAngles[0];
			msg.name[1] = "joint_thumb2_thumb3";
			msg.position[1] = actualAngles[1];
			msg.name[2] = "joint_palm_finger11";
			msg.position[2] = actualAngles[2];
			msg.name[3] = "joint_finger11_finger12";
			msg.position[3] = actualAngles[3];
			msg.name[4] = "joint_finger12_finger13";
			msg.position[4] = actualAngles[4];
			msg.name[5] = "joint_palm_finger21";
			msg.position[5] = actualAngles[5];
			msg.name[6] = "joint_finger21_finger22";
			msg.position[6] = actualAngles[6];
			msg.name[7] = "joint_finger22_finger23";
			msg.position[7] = 4.0;

			//publish the message
			topicPub_JointState.publish(msg);

			//ROS_INFO("published JointState 3");
			/*
			if(isInitialized == true)
			{
				printf("angles %f %f %f %f %f\n", actualAngles[0], actualAngles[1], actualAngles[2], actualAngles[3], actualAngles[4]);
			}
			*/
		}

		/* ////
		   void updateTactileData()
		   {
		   ROS_INFO("updateTactileData");
		   cob_msgs::TactileMatrix msg;
		   if(isDSAInitialized)
		   {
		   dsa->UpdateFrame();
		   unsigned int m, x, y;
		   for ( m = 0; m < dsa->GetSensorInfo().nb_matrices; m++ )
		   {
		   msg.maxtrix_id = m;
		   int cells_y = dsa->GetMatrixInfo( m ).cells_y;
		   int cells_x = dsa->GetMatrixInfo( m ).cells_x;
		   msg.cells_y = cells_y;
		   msg.cells_x = cells_x;
		   msg.texel_data.resize((cells_y*cells_x)+1);
		   for ( y = 0; y < cells_y; y++ )
		   {
		   for ( x = 0; x < cells_x; x++ )
		   {
		   msg.texel_data[(y+1)*(x+1)] = dsa->GetTexel( m, x, y );
		//std::cout << std::setw( 4 ) << dsa->GetTexel( m, x, y ) << " ";
		}
		//std::cout << "\n";
		}
		//std::cout << "\n\n";
		//publish matrix
		topicPub_TactileMatrix.publish(msg);
		}
		}
		}
		 */
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "sdh");
	ROS_INFO("...sdh node running...");

	NodeClass nodeClass;
	sleep(1);
	ros::Rate loop_rate(5); // Hz
	while(nodeClass.n.ok())
	{
		// publish JointState
		nodeClass.updateJointState();
		////nodeClass.updateTactileData();

		// sleep and waiting for messages, callbacks    
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
