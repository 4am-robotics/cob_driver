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
 * ROS package name: cob_base_drive_chain
 * Description: This node provides control of the care-o-bot platform drives to the ROS-"network". For this purpose it offers several services and publishes data on different topics.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2010:
 * ToDo: Doesn't this node has to take care about the Watchdogs?
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
#include <sstream>
#include <iostream>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/GetJointState.h>
#include <cob_base_drive_chain/ElmoRecorderGet.h>
#include <cob_base_drive_chain/ElmoRecorderSetup.h>


// external includes
#include <cob_base_drive_chain/CanCtrlPltfCOb3.h>
#include <cob_utilities/IniFile.h>
#include <cob_utilities/MathSup.h>

//####################
//#### node class ####
/**
* This node provides control of the care-o-bot platform drives to the ROS-"network". For this purpose it offers several services and publishes data on different topics.
*/
class NodeClass
{
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        /**
        * On this topic "JointState" of type sensor_msgs::JointState the node publishes joint states when they are requested over the appropriate service srvServer_GetJointState.
        */
        ros::Publisher topicPub_JointState;
        
        /**
        * On this topic "Diagnostic" of type diagnostic_msgs::DiagnosticStatus the node publishes states and error information about the platform.
        */
		ros::Publisher topicPub_Diagnostic;
        
        
	    // topics to subscribe, callback is called for new messages arriving
		/**
        * The node subscribes to the topic "JointStateCmd" and performs the requested motor commands
        */
		ros::Subscriber topicSub_JointStateCmd;
        
        // service servers
        /**
        * Service requests cob_srvs::Trigger and initializes platform and motors
        */
        ros::ServiceServer srvServer_Init;
        
        /**
        * Service requests cob_srvs::Trigger and resets platform and motors
        */
        ros::ServiceServer srvServer_Reset;
        
        /**
        * Service requests cob_srvs::Trigger and shuts down platform and motors
        */
		ros::ServiceServer srvServer_Shutdown;

		ros::ServiceServer srvServer_SetMotionType;
		
		/**
        * Service requests cob_srvs::GetJointState. It reads out the latest joint information from the CAN buffer and gives it back. It also publishes the informaion on the topic "JointState"
        */
		ros::ServiceServer srvServer_GetJointState;
		
		/**
        * Service requests cob_base_drive_chain::ElmoRecorderSetup. It is used to configure the Elmo Recorder to record predefined sources. 
        * Parameters are:
        * int64 recordinggap #Specify every which time quantum (4*90usec) a new data point (of 1024 points in total) is recorded. the recording process starts immediately.
        */
		ros::ServiceServer srvServer_ElmoRecorderConfig;
		
		/**
        * Service requests cob_base_drive_chain::ElmoRecorderGet. It is used to start the read-out process of previously recorded data by the Elmo Recorder.
        * Parameters are:
        * int64 subindex 
        * #As Subindex, set the recorded source you want to read out:
		* #1: Main Speed
		* #2: Main Position
		* #10: ActiveCurrent
		* #16: Speed Command
		*
		* string fileprefix
		* #Enter the path+file-prefix for the logfile (of an existing directory!)
		* #The file-prefix is extended with _MotorNumber_RecordedSource.log
        */
		ros::ServiceServer srvServer_ElmoRecorderReadout;

        // global variables
		// generate can-node handle
		CanCtrlPltfCOb3 m_CanCtrlPltf;
		bool m_bisInitialized;
		int m_iNumMotors;
        
		struct ParamType
		{ 
			double dMaxDriveRateRadpS;
			double dMaxSteerRateRadpS;

			std::vector<double> vdWheelNtrlPosRad;
		};
		ParamType m_Param;
		std::string sIniDirectory;

		// Constructor
		NodeClass()
		{
			// initialization of variables
			m_bisInitialized = false;
			m_iNumMotors = 8;
			
			// implementation of topics
			// published topics
			topicPub_JointState = n.advertise<sensor_msgs::JointState>("JointState", 1);
			topicPub_Diagnostic = n.advertise<diagnostic_msgs::DiagnosticStatus>("Diagnostic", 1);
			// subscribed topics
			topicSub_JointStateCmd = n.subscribe("JointStateCmd", 1, &NodeClass::topicCallback_JointStateCmd, this);

			// implementation of service servers
			srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
            srvServer_ElmoRecorderConfig = n.advertiseService("ElmoRecorderConfig", &NodeClass::srvCallback_ElmoRecorderConfig, this);
            srvServer_ElmoRecorderReadout = n.advertiseService("ElmoRecorderReadout", &NodeClass::srvCallback_ElmoRecorderReadout, this);
            
			srvServer_Reset = n.advertiseService("Reset", &NodeClass::srvCallback_Reset, this);
			srvServer_Shutdown = n.advertiseService("Shutdown", &NodeClass::srvCallback_Shutdown, this);
			//srvServer_isPltfError = n.advertiseService("isPltfError", &NodeClass::srvCallback_isPltfError, this); --> Publish this along with JointStates
			srvServer_GetJointState = n.advertiseService("GetJointState", &NodeClass::srvCallback_GetJointState, this);
		}
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 
        // function will be called when a new message arrives on a topic
        void topicCallback_JointStateCmd(const sensor_msgs::JointState::ConstPtr& msg)
        {
			ROS_DEBUG("Topic Callback JointStateCmd");
			// only process cmds when system is initialized
			if(m_bisInitialized == true)
			{
		   		int iRet;
				sensor_msgs::JointState JointStateCmd = *msg;
            	// check if velocities lie inside allowed boundaries
		    	for(int i = 0; i < m_iNumMotors; i++)
		    	{
				    // for steering motors
            	    if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the config-files
            	    {
				        if (JointStateCmd.velocity[i] > m_Param.dMaxSteerRateRadpS)
				        {
				        	JointStateCmd.velocity[i] = m_Param.dMaxSteerRateRadpS;
				        }
				        if (JointStateCmd.velocity[i] < -m_Param.dMaxSteerRateRadpS)
				        {
				    	    JointStateCmd.velocity[i] = -m_Param.dMaxSteerRateRadpS;
				        }
            	    }
            	    else    // for driving motors
				    if (JointStateCmd.velocity[i] > m_Param.dMaxDriveRateRadpS)
				    {
				    	JointStateCmd.velocity[i] = m_Param.dMaxDriveRateRadpS;
				    }
				    if (JointStateCmd.velocity[i] < -m_Param.dMaxDriveRateRadpS)
					{
				    	JointStateCmd.velocity[i] = -m_Param.dMaxDriveRateRadpS;
			    	}

                	// and cmd velocities to Can-Nodes
                	//m_CanCtrlPltf.setVelGearRadS(iCanIdent, dVelEncRadS);
                	iRet = m_CanCtrlPltf.setVelGearRadS(i, JointStateCmd.velocity[i]);
      	    	}
			}
        }

        // service callback functions
        // function will be called when a service is querried

		// Init Can-Configuration
        bool srvCallback_Init(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
			ROS_DEBUG("Service Callback Init");
            if(m_bisInitialized == false)
            {
                m_bisInitialized = initDrives();
                //ROS_INFO("...initializing can-nodes...");
		        //m_bisInitialized = m_CanCtrlPltf.initPltf();
		        res.success = m_bisInitialized;
                if(m_bisInitialized)
		        {
           	        ROS_INFO("Can-Node initialized");
		        }
		        else
		        {
                    res.errorMessage.data = "initialization of can-nodes failed";
                  	ROS_INFO("Initialization FAILED");
		        }
            }
            else
            {
                ROS_ERROR("...platform already initialized...");
                res.success = false;
                res.errorMessage.data = "platform already initialized";
            }            
            return true;
        }
		
		bool srvCallback_ElmoRecorderConfig(cob_base_drive_chain::ElmoRecorderSetup::Request &req,
                              cob_base_drive_chain::ElmoRecorderSetup::Response &res ){
			if(m_bisInitialized) {			
				m_CanCtrlPltf.evalCanBuffer();
				res.success = m_CanCtrlPltf.ElmoRecordings(0, req.recordinggap, "");
				res.message = "Successfully configured all motors for instant record";
			}

			return true;
		}
		
		bool srvCallback_ElmoRecorderReadout(cob_base_drive_chain::ElmoRecorderGet::Request &req,
                              cob_base_drive_chain::ElmoRecorderGet::Response &res ){
			if(m_bisInitialized) {
				m_CanCtrlPltf.evalCanBuffer();
				res.success = m_CanCtrlPltf.ElmoRecordings(1, req.subindex, req.fileprefix);
				if(res.success == 0) res.message = "Successfully requested reading out of Recorded data";
				else if(res.success == 1) res.message = "Recorder hasn't been configured well yet";
				else if(res.success == 2) res.message = "A previous transmission is still in progress";
			}

			return true;
		}
		
		
		
		// reset Can-Configuration
        bool srvCallback_Reset(cob_srvs::Trigger::Request &req,
                                     cob_srvs::Trigger::Response &res )
        {
			ROS_DEBUG("Service Callback Reset");
	    	res.success = m_CanCtrlPltf.resetPltf();
		    if (res.success)
       	        ROS_INFO("Can-Node resetted");
		    else
                res.errorMessage.data = "reset of can-nodes failed";
            	ROS_INFO("Reset of Can-Node FAILED");

		    return true;
        }
		
		// shutdown Drivers and Can-Node
        bool srvCallback_Shutdown(cob_srvs::Trigger::Request &req,
                                     cob_srvs::Trigger::Response &res )
        {
			ROS_DEBUG("Service Callback Shutdown");
	    	res.success = m_CanCtrlPltf.shutdownPltf();
	    	if (res.success)
       	    	ROS_INFO("Drives shut down");
	    	else
       	    	ROS_INFO("Shutdown of Drives FAILED");

	    	return true;
        }

        bool srvCallback_GetJointState(cob_srvs::GetJointState::Request &req,
                                     cob_srvs::GetJointState::Response &res )
        {
			ROS_DEBUG("Service Callback GetJointState");
            // init local variables
            int iCanEvalStatus, ret, j, k;
            bool bIsError;
            std::vector<double> vdAngGearRad, vdVelGearRad, vdEffortGearNM;
			std::string str_steer, str_drive, str_cat;
			std::stringstream str_num;

			// init strings
			str_steer = "Steer";
			str_drive = "Drive";

            // set default values
            vdAngGearRad.resize(m_iNumMotors, 0);
            vdVelGearRad.resize(m_iNumMotors, 0);
            vdEffortGearNM.resize(m_iNumMotors, 0);

            // create temporary (local) JointState/Diagnostics Data-Container
            sensor_msgs::JointState jointstate;
            diagnostic_msgs::DiagnosticStatus diagnostics;
			

			//Do you have to set frame_id manually??

			// get time stamp for header
			jointstate.header.stamp = ros::Time::now();
            // set frame_id for header            
			// jointstate.header.frame_id = frame_id; //Where to get this id from?

			// assign right size to JointState
			jointstate.set_name_size(m_iNumMotors);
            jointstate.set_position_size(m_iNumMotors);
            jointstate.set_velocity_size(m_iNumMotors);            
            jointstate.set_effort_size(m_iNumMotors);

            if(m_bisInitialized == false)
            {
				// as long as system is not initialized
				bIsError = false;

	            // set data to jointstate            
	            for(int i = 0; i<m_iNumMotors; i++)
	            {
	                jointstate.position[i] = 0.0;
	                jointstate.velocity[i] = 0.0;
	                jointstate.effort[i] = 0.0;
	            }
			}
			else
			{
				// as soon as drive chain is initialized
	            // read Can-Buffer
    			iCanEvalStatus = m_CanCtrlPltf.evalCanBuffer();
    	        
    	        j = 0;
				k = 0;
    	        for(int i = 0; i<m_iNumMotors; i++)
    	        {
		    		ret = m_CanCtrlPltf.getGearPosVelRadS(i,  &vdAngGearRad[i], &vdVelGearRad[i]);
   	            	// if a steering motor was read -> correct for offset
   	            	if( i == 1 || i == 3 || i == 5 || i == 7) // ToDo: specify this via the config-files
                	{
                    	// correct for initial offset of steering angle (arbitrary homing position)
		            	vdAngGearRad[i] += m_Param.vdWheelNtrlPosRad[j];
	                	MathSup::normalizePi(vdAngGearRad[i]);
                    	j = j+1;
						// create name for identification in JointState msg
						str_num << j;
						str_cat = str_steer + str_num.str();
                	}
					else
					{
						// create name for identification in JointState msg
						k = k+1;
						str_num << k;
						str_cat = str_drive + str_num.str();
					}
					// set joint names
					jointstate.name[i] = str_cat;
            	}

            	// set data to jointstate            
            	for(int i = 0; i<m_iNumMotors; i++)
            	{
            	    jointstate.position[i] = vdAngGearRad[i];
            	    jointstate.velocity[i] = vdVelGearRad[i];
            	    jointstate.effort[i] = vdEffortGearNM[i];
            	}
			}

            // set answer to srv request
            res.jointstate = jointstate;

        	// publish jointstate message
            topicPub_JointState.publish(jointstate);
        	ROS_DEBUG("published new drive-chain configuration (JointState message)");
			

            if(m_bisInitialized)
            {
				// read Can only after initialization
	    		bIsError = m_CanCtrlPltf.isPltfError();
			}

            // set data to diagnostics
            if(bIsError)
            {
                diagnostics.level = 2;
                diagnostics.name = "drive-chain can node";
                diagnostics.message = "one or more drives are in Error mode";
            }
            else
            {
				if (m_bisInitialized)
				{
                	diagnostics.level = 0;
                	diagnostics.name = "drive-chain can node";
                	diagnostics.message = "drives operating normal";
				}
				else
				{
                	diagnostics.level = 1;
                	diagnostics.name = "drive-chain can node";
                	diagnostics.message = "drives are initializing";
				}
            }

            // publish diagnostic message
            topicPub_Diagnostic.publish(diagnostics);
        	ROS_DEBUG("published new drive-chain configuration (JointState message)");

            return true;
        }
        
        // other function declarations
        bool initDrives();
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "base_drive_chain");
    
	NodeClass nodeClass;
	
	ros::Time time_evalcan_buffer = ros::Time::now();
    
	while(nodeClass.n.ok())
	{
		// Read out the CAN buffer only every n seconds; cycle the loop without any sleep time to make services available at all time.
		if(ros::Time::now().toSec() - time_evalcan_buffer.toSec() > 0.001) {
			if(nodeClass.m_bisInitialized) nodeClass.m_CanCtrlPltf.evalCanBuffer();
			//Read-out of CAN buffer is especially necessary during read-out of Elmo Recorder
			time_evalcan_buffer = ros::Time::now();
		}
		ros::spinOnce();
	}
	return 0;
}

//##################################
//#### function implementations ####
bool NodeClass::initDrives()
{
    ROS_INFO("Initializing Base Drive Chain");

    // init member vectors
	m_Param.vdWheelNtrlPosRad.assign(4,0);

    // ToDo: replace the following steps by ROS configuration files
    // create Inifile class and set target inifile (from which data shall be read)
	IniFile iniFile;

	/// Parameters are set within the launch file
	n.param<std::string>("base_drive_chain_node/IniDirectory", sIniDirectory, "Platform/IniFiles/");
	ROS_INFO("IniDirectory loaded from Parameter-Server is: %s", sIniDirectory.c_str());
	

    //n.param<std::string>("PltfIniLoc", sIniFileName, "Platform/IniFiles/Platform.ini");
	iniFile.SetFileName(sIniDirectory + "Platform.ini", "PltfHardwareCoB3.h");

    // get max Joint-Velocities (in rad/s) for Steer- and Drive-Joint
	iniFile.GetKeyDouble("DrivePrms", "MaxDriveRate", &m_Param.dMaxDriveRateRadpS, true);
	iniFile.GetKeyDouble("DrivePrms", "MaxSteerRate", &m_Param.dMaxSteerRateRadpS, true);
	
    // get Offset from Zero-Position of Steering	
	iniFile.GetKeyDouble("DrivePrms", "Wheel1NeutralPosition", &m_Param.vdWheelNtrlPosRad[0], true);
	iniFile.GetKeyDouble("DrivePrms", "Wheel2NeutralPosition", &m_Param.vdWheelNtrlPosRad[1], true);
	iniFile.GetKeyDouble("DrivePrms", "Wheel3NeutralPosition", &m_Param.vdWheelNtrlPosRad[2], true);
	iniFile.GetKeyDouble("DrivePrms", "Wheel4NeutralPosition", &m_Param.vdWheelNtrlPosRad[3], true);

	//Convert Degree-Value from ini-File into Radian:
	m_Param.vdWheelNtrlPosRad[0] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[0]);
	m_Param.vdWheelNtrlPosRad[1] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[1]);
	m_Param.vdWheelNtrlPosRad[2] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[2]);
	m_Param.vdWheelNtrlPosRad[3] = MathSup::convDegToRad(m_Param.vdWheelNtrlPosRad[3]);

	// debug log
	ROS_INFO("Initializing CanCtrlItf");
	bool bTemp1;
	bTemp1 =  m_CanCtrlPltf.initPltf(sIniDirectory);
	// debug log
	ROS_INFO("Initializing done");


	return bTemp1;
}
