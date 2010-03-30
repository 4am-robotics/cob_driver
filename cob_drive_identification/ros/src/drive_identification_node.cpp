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
 * ROS stack name: cob3_apps
 * ROS package name: drive_identification
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: cpc-pk
 * Supervised by: cpc
 *
 * Date of creation: Mar 2010
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
// Headers provided by cob-packages which should be avoided/removed
#include <cob3_utilities/IniFile.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
//#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

// ROS service includes
#include <std_srvs/Empty.h>

// external includes
//--

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	    // create a handle for this node, initialize node
	    ros::NodeHandle n;
                
        // topics to publish
        ros::Publisher topicPub_JointStateCmd;
        
	    // topics to subscribe, callback is called for new messages arriving
        ros::Subscriber topicSub_demoSubscribe;
        
        // service servers
        //--
            
        // service clients
        ros::ServiceClient srvClient_GetJointState<cob3_srvs::GetJointState>("GetJointState");
        ros::ServiceClient srvClient_InitPltf<cob3_srvs::Switch>("Init");
        ros::ServiceClient srcClient_ShutdownPltf<cob3_srvs::Switch>("Shutdown");
        
        // global variables
        std::vector<double> m_vdVelGearDriveRadS;
	    std::vector<double> m_vdVelGearSteerRadS;
	    double m_dSpeedRadS;
		std::string m_sFilePrefix;
        int iNumMotors;        

        // Types of Identification Modes
		enum IdentModus
		{
			IdentDrives, IdentSteers
		};

        // Constructor
        NodeClass()
        {
            topicPub_JointStateCmd = n.advertise<sensor_msgs::JointState>("JointStateCmd", 1);
            //topicSub_demoSubscribe = n.subscribe("demoSubscribe", 1, &NodeClass::topicCallback_demoSubscribe, this);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }

        // topic callback functions 

        // service callback functions

        
        // other function declarations
        bool startupIdentification();
        
        bool startDriveIdentification(IdentModus mode);
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "drive_identification");
    
    NodeClass identification;
    identification.startupIdentification();
 
    while(identification.n.ok())
    {

        ros::spinOnce();
    }
    
//    ros::spin();

    return 0;
}

//##################################
//#### function implementations ####
bool NodeClass::startupIdentification(){
    srvClient_InitPltf.call(cob3_srvs::Switch data);
    if(data.response.success != true) {
        ROS_ERROR("Failed to initialize Platform with message: %s", data.response.errorMessage);
        return 0;
    }   

    //Read from ini-File
    iNumMotors = 8; //Parameter??!
    m_vdVelGearDriveRadS.assign(4,0);
    m_vdVelGearSteerRadS.assign(4,0);
    m_speedRadS = 6.0;
	m_sFilePrefix = "Platform\\Log\\ElmoRecordings";
    
    IniFile iniFile;
	iniFile.SetFileName("Platform/IniFiles/PltfIdent.ini", "drive_identification_node.cpp");
	iniFile.GetKeyDouble("Identification", "SpeedRadS", &m_dSpeedRadS, true);
	iniFile.GetKeyString("Identification", "FilePrefix", &m_sFilePrefix, true);

    return 0;
}

bool startDriveIdentification(IdentModus mode){
    double dVelDrivesLeft=0, dVelDrivesRight=0;
    double dVelSteers = 0;
    sensor_msgs::JointState msgDriveCmd;
    cob3_srvs::GetJointState srvGetJointState;
    sensor_msgs::JointState jointstate;
    
    // Declare an array of two Vectors two double values (Time, Velocity)
    std::vector<double> vWheel1_Drive[2]; std::vector<double> vWheel1_Steer[2];
	std::vector<double> vWheel2_Drive[2]; std::vector<double> vWheel2_Steer[2];
	std::vector<double> vWheel3_Drive[2]; std::vector<double> vWheel3_Steer[2];
	std::vector<double> vWheel4_Drive[2]; std::vector<double> vWheel4_Steer[2];

    ROS_INFO("Start drive identification");
	switch (identModus)
	{
		case IdentDrives:
				dVelDrivesLeft=-m_dSpeedRadS;
				dVelDrivesRight=m_dSpeedRadS;
				dVelSteers=0.0;
			break;
		case IdentSteers:
				dVelDrivesLeft=0;
				dVelDrivesRight=0;
				dVelSteers=m_dSpeedRadS;
			break;
	}

    double dDeltaTime, dTimeNow;
    double dTimeStart = ros::Time::now().toSec();

   	// 2. Send & Receive data from drives
	bool bReadyForNextStep = false;
	do 
	{
		dTimeNow = ros::Time::now().toSec();
		dDeltaTime = dTimeNow - dTimeStart;
		if ( (dDeltaTime > 3.0) &&  (dDeltaTime < 6.0)) 
		{	// After 3 Seconds passed -> set velocity to zero

            for(int i = 0; i<iNumMotors; i++) {
                msgDriveCmd.velocity[i] = 0;
            }
            pubTopic_JointStateCmd.publish(driveCmd);

			/*
            m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL1STEERMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL1DRIVEMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL2STEERMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL2DRIVEMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL3STEERMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL3DRIVEMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL4STEERMOTOR, 0);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL4DRIVEMOTOR, 0);*/
		}
		else if ( (dDeltaTime >= 6.0) )
		{	// After 6 Seconds stop recording (go to next step)
			bReadyForNextStep = true;
		}
		else
		{
			// Command velocities, elmos will autorespond with velocities and positions
            
            
            msgDriveCmd.velocity[1] = dVelSteers; //Motors are named counterclockwise (mathematical positive)
            msgDriveCmd.velocity[3] = dVelSteers;
            msgDriveCmd.velocity[5] = dVelSteers;
            msgDriveCmd.velocity[7] = dVelSteers;
            msgDriveCmd.velocity[0] = dVelDrivesLeft;
            msgDriveCmd.velocity[2] = dVelDrivesLeft;
            msgDriveCmd.velocity[4] = dVelDrivesRight;
            msgDriveCmd.velocity[6] = dVelDrivesRight;

            pubTopic_JointStateCmd.publish(driveCmd);

            /*
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL1STEERMOTOR, dVelSteers);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL1DRIVEMOTOR, dVelDrivesLeft);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL2STEERMOTOR, dVelSteers);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL2DRIVEMOTOR, dVelDrivesLeft);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL3STEERMOTOR, dVelSteers);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL3DRIVEMOTOR, dVelDrivesRight);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL4STEERMOTOR, dVelSteers);
			m_pCanCtrlPltfCoB3->setVelGearRadS(m_pCanCtrlPltfCoB3->CANNODE_WHEEL4DRIVEMOTOR, dVelDrivesRight);*/
		}

		// Calculate current time of identification process
		dDeltaTime = ros::Time::now().toSec() - dTimeStart;

   		//Sleep(3); HOW LONG IS THAT SLEEP?
        ros::Duration(0.003).sleep();
		
        // Request "real" joint state from base_drive_chain
        srvClient_GetJointState(srvGetJointState);
        
		// Read and save new values
		//vWheel1_Drive[0].push_back( dDeltaTime );
		//vWheel1_Drive[1].push_back( m_vdVelGearDriveRadS[0] );
        vWheel1_Drive[0].push_back( dDeltaTime ); 
        vWheel1_Drive[1].push_back( jointstate.velocity[0] ); //Drives have even numbers 
		vWheel1_Steer[0].push_back( dDeltaTime );
		vWheel1_Steer[1].push_back( jointstate.velocity[1] );


		vWheel2_Drive[0].push_back( dDeltaTime );
		vWheel2_Drive[1].push_back( jointstate.velocity[2] );
		vWheel2_Steer[0].push_back( dDeltaTime );
		vWheel2_Steer[1].push_back( jointstate.velocity[3] );

		vWheel3_Drive[0].push_back( dDeltaTime );
		vWheel3_Drive[1].push_back( jointstate.velocity[4] );
		vWheel3_Steer[0].push_back( dDeltaTime );
		vWheel3_Steer[1].push_back( jointstate.velocity[5] );
	
		vWheel4_Drive[0].push_back( dDeltaTime );
		vWheel4_Drive[1].push_back( jointstate.velocity[6] );
		vWheel4_Steer[0].push_back( dDeltaTime );
		vWheel4_Steer[1].push_back( jointstate.velocity[7] );				
		
	} while (!bReadyForNextStep)
}
