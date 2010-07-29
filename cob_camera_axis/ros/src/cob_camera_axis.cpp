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
 * ROS package name: cob_camera_axis
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Ulrich Reiser, email:ulrich.reiser@ipa.fhg.de
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
#include <urdf/model.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <cob_msgs/JointCommand.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

// external includes
#include <cob_camera_axis/ElmoCtrl.h>

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
	
	    // declaration of topics to subscribe, callback is called for new messages arriving
	ros::Subscriber topicSub_JointCommand;
	
	// declaration of service servers
	ros::ServiceServer srvServer_Init;
	ros::ServiceServer srvServer_Stop;
	ros::ServiceServer srvServer_Recover;
	ros::ServiceServer srvServer_SetOperationMode;
	    
	// declaration of service clients
	//--
	
	// global variables
	ElmoCtrl * CamAxis;
		ElmoCtrlParams* CamAxisParams;
		std::string CanDevice;
		std::string CanIniFile;
		int CanBaudrate;
		int HomingDir;
		double MaxVel;
		int ModID;
		double LowerLimit;
		double UpperLimit; 
		double Offset;
		std::string JointName;
		bool isInitialized;

	// Constructor
	NodeClass()
	{
			isInitialized = false;

		CamAxis = new ElmoCtrl();
		CamAxisParams = new ElmoCtrlParams();

	    // implementation of topics to publish
	    topicPub_JointState = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	    
	    // implementation of topics to subscribe
	    topicSub_JointCommand = n.subscribe("joint_commands", 1, &NodeClass::topicCallback_JointCommand, this);
	    
	    // implementation of service servers
	    srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
	    srvServer_Stop = n.advertiseService("Stop", &NodeClass::srvCallback_Stop, this);
	    srvServer_Recover = n.advertiseService("Recover", &NodeClass::srvCallback_Recover, this);
	    
	    // implementation of service clients
	    //--

	    // read parameters from parameter server
	    n.param<std::string>("CanDevice", CanDevice,"ESD:1");
	    n.param<int>("CanBaudrate", CanBaudrate, 500);
	    n.param<int>("HomingDir", HomingDir, -1);
	    n.param<int>("ModId",ModID ,17 );
	    n.param<std::string>("JointName",JointName ,"joint_head_eyes" );
	    n.param<std::string>("CanIniFile",CanIniFile ,"IniFiles/CanCtrlCamAxis.ini" );
	    n.param<double>("MaxVel", MaxVel, 2.0);
		ROS_INFO("CanDevice=%s, CanBaudrate=%d,ModID=%d",CanDevice.c_str(),CanBaudrate,ModID);
		
		CamAxisParams->SetCanIniFile(CanIniFile);
		CamAxisParams->SetHomingDir(HomingDir);
		CamAxisParams->SetMaxVel(MaxVel);


		CamAxisParams->Init(CanDevice, CanBaudrate, ModID);
		
		
		// load parameter server string for robot/model
		std::string param_name = "robot_description";
		std::string full_param_name;
		std::string xml_string;
		n.searchParam(param_name,full_param_name);
		n.getParam(full_param_name.c_str(),xml_string);
		ROS_INFO("full_param_name=%s",full_param_name.c_str());
		if (xml_string.size()==0)
		{
			ROS_ERROR("Unable to load robot model from param server robot_description\n");
			exit(2);
		}
		ROS_DEBUG("%s content\n%s", full_param_name.c_str(), xml_string.c_str());
		
		// extract limits and velocitys from urdf model
		urdf::Model model;
		if (!model.initString(xml_string))
		{
			ROS_ERROR("Failed to parse urdf file");
			exit(2);
		}
		ROS_INFO("Successfully parsed urdf file");



		//TODO: check if yaml parameter file fits to urdf model
		// get LowerLimits out of urdf model
		LowerLimit = model.getJoint(JointName.c_str())->limits->lower;
			//std::cout << "LowerLimits[" << JointNames[i].c_str() << "] = " << LowerLimits[i] << std::endl;
		CamAxisParams->SetLowerLimit(LowerLimit);

		// get UpperLimits out of urdf model
		UpperLimit = model.getJoint(JointName.c_str())->limits->upper;
			//std::cout << "LowerLimits[" << JointNames[i].c_str() << "] = " << LowerLimits[i] << std::endl;
		CamAxisParams->SetUpperLimit(UpperLimit);

		// get Offset out of urdf model
		Offset = model.getJoint(JointName.c_str())->calibration->reference_position;
			//std::cout << "Offset[" << JointNames[i].c_str() << "] = " << Offsets[i] << std::endl;
		CamAxisParams->SetAngleOffset(Offset);
		
		
		std::cout << "Upper Limit = " << CamAxisParams->GetUpperLimit() << " Lower Limit: " << CamAxisParams->GetLowerLimit() << std::endl;
	}
	
	// Destructor
	~NodeClass() 
	{
	}

	// topic callback functions 
	// function will be called when a new message arrives on a topic
	void topicCallback_JointCommand(const cob_msgs::JointCommand::ConstPtr& msg)
	{
   			if (isInitialized == true)
			{
				std::cout << "Setting Position to " << msg->positions[0] << " at speed " << msg->velocities[0] << std::endl;
				CamAxis->setGearPosVelRadS((msg->positions)[0],(msg->velocities)[0]);
			}
			else	
			{
				ROS_ERROR("camera axis not initialized");
			}
	}

	// service callback functions
	// function will be called when a service is querried
	bool srvCallback_Init(cob_srvs::Trigger::Request &req,
			      cob_srvs::Trigger::Response &res )
	{
			if (isInitialized == false)
			{
				ROS_INFO("...initializing camera axis...");
		      	// init powercubes 
			if (CamAxis->Init(CamAxisParams))
			{
				ROS_INFO("Initializing of camera axis succesful");
					isInitialized = true;
				res.success = 0; // 0 = true, else = false
			}
			else
			{
				ROS_ERROR("Initializing camera axis not succesful \n");
				//res.success = 1; // 0 = true, else = false
				//res.errorMessage.data = PCube->getErrorMessage();
			}
			}
			else
			{
				ROS_ERROR("...camera axis already initialized...");			
				res.success = 1;
				res.errorMessage.data = "camera axis already initialized";
			}

		// homing powercubes
		/*if (PCube->doHoming())
		{
			ROS_INFO("Homing powercubes succesfull");
			res.success = 0; // 0 = true, else = false
		}
		else
		{
			ROS_ERROR("Homing powercubes not succesfull. error: %s", PCube->getErrorMessage().c_str());
			res.success = 1; // 0 = true, else = false
			res.errorMessage.data = PCube->getErrorMessage();
			return true;
		}
			*/

		     return true;
	}

	bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
			      cob_srvs::Trigger::Response &res )
	{
       	    ROS_INFO("Stopping camera axis");
	
	    // stopping all arm movements
	    if (CamAxis->Stop())
	    {
	    	ROS_INFO("Stopping camera axis succesful");
	    	res.success = 0; // 0 = true, else = false
	    }
	    else
	    {
	    	ROS_ERROR("Stopping camera axis not succesful. error");
	    }
	    return true;
	}
	
	bool srvCallback_Recover(cob_srvs::Trigger::Request &req,
			      	 cob_srvs::Trigger::Response &res )
	{
			if (isInitialized == true)
			{
		   	    ROS_INFO("Recovering camera axis");
		    
			// stopping all arm movements
			if (CamAxis->Stop())
			{
				ROS_INFO("Recovering camera axis succesful");
				res.success = 0; // 0 = true, else = false
			}
			else
			{
				ROS_ERROR("Recovering camera axis not succesful. error");
			}
		    }
		    else
			{
				ROS_ERROR("...camera axis already recovered...");			
			}

	    return true;
	}
/*
	bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
					  cob_srvs::SetOperationMode::Response &res )
	{
		ROS_INFO("Set operation mode to [%s]", req.operationMode.data.c_str());
	    n.setParam("OperationMode", req.operationMode.data.c_str());
	    res.success = 0; // 0 = true, else = false
	    return true;
	}
  */	      
	// other function declarations
	void publishJointState()
	{
		if (isInitialized == true)
		{
			//CamAxis->evalCanBuffer();
			
			// create message
			int DOF = 1;
			double ActualPos;
			double ActualVel;
			ActualPos=0.0;
			ActualVel=0.0;

			CamAxis->getGearPosVelRadS(&ActualPos,&ActualVel);

			sensor_msgs::JointState msg;
			msg.header.stamp = ros::Time::now();
			msg.name.resize(DOF);
			msg.position.resize(DOF);
			msg.velocity.resize(DOF);
			
			msg.name[0] = JointName;
			msg.position[0] = ActualPos;
			msg.velocity[0] = ActualVel;


			std::cout << "Joint " << msg.name[0] <<": pos="<<  msg.position[0] << " vel=" << msg.velocity[0] << std::endl;
			    
			// publish message
			topicPub_JointState.publish(msg);
		}
	}

}; //NodeClass

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "cob_camera_axis");
    
    // create nodeClass
    NodeClass nodeClass;
 
    // main loop
 	ros::Rate loop_rate(10); // Hz
    while(nodeClass.n.ok())
    {
      
	// publish JointState
	//nodeClass.publishJointState();

	// read parameter
	//std::string operationMode;
	//nodeClass.n.getParam("OperationMode", operationMode);
	//ROS_DEBUG("running with OperationMode [%s]", operationMode.c_str());

	// sleep and waiting for messages, callbacks 
	ros::spinOnce();
	loop_rate.sleep();
    }
    
//    ros::spin();

    return 0;
}
