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

//#define USE_ESD

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <urdf/model.h>
#include <actionlib/server/simple_action_server.h>
//#include <cob_actions/JointTrajectoryAction.h>
#include <cob_actions/JointCommandAction.h>

// ROS message includes
#include <cob_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
//#include <trajectory_msgs/JointTrajectory.h>
//// #include <cob_msgs/TactileMatrix.h>

// ROS service includes
#include <cob_srvs/Trigger.h>
#include <cob_srvs/SetOperationMode.h>

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
		//ros::Publisher topicPub_ActuatorState;
		////ros::Publisher topicPub_TactileMatrix;

		// declaration of topics to subscribe, callback is called for new messages arriving
		//ros::Subscriber topicSub_JointCommand;

		// service servers
		ros::ServiceServer srvServer_Init;
        ros::ServiceServer srvServer_Stop;
        ros::ServiceServer srvServer_SetOperationMode;

        // action lib server
		//actionlib::SimpleActionServer<cob_actions::JointTrajectoryAction> as_;
		actionlib::SimpleActionServer<cob_actions::JointCommandAction> as_;
		std::string action_name_;
		// create messages that are used to published feedback/result
		//cob_actions::JointTrajectoryFeedback feedback_;
		//cob_actions::JointTrajectoryResult result_;
		cob_actions::JointCommandFeedback feedback_;
		cob_actions::JointCommandResult result_;

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
		int DOF_HW,DOF_ROS;
		double pi_;
		
		//trajectory_msgs::JointTrajectory traj;
		//trajectory_msgs::JointTrajectoryPoint traj_point;
		//int traj_point_nr;
		
		cob_msgs::JointCommand command;
		
		XmlRpc::XmlRpcValue JointNames_param;
		XmlRpc::XmlRpcValue JointNamesAll_param;
		std::vector<std::string> JointNames;
		std::vector<std::string> JointNamesAll;
		std::vector<int> axes_;

		// Constructor
		NodeClass(std::string name):
			as_(n, name, boost::bind(&NodeClass::executeCB, this, _1)),
			action_name_(name)
		{
			// initialize global variables
			isInitialized = false;
			isDSAInitialized = false;
			
			pi_ = 3.1415926;

			// implementation of topics to publish
			topicPub_JointState = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
			////topicPub_TactileMatrix = n.advertise<cob_msgs::TactileMatrix>("tactile_data", 1);

			// implementation of topics to subscribe

#ifdef USE_ESD
			n.param("sdhdevicetype", sdhdevicetype, std::string("ESD"));
			n.param("sdhdevicestring", sdhdevicestring, std::string("/dev/can0"));
#else
			n.param("sdhdevicetype", sdhdevicetype, std::string("PEAK"));
			n.param("sdhdevicestring", sdhdevicestring, std::string("/dev/pcan0"));
#endif
			n.param("sdhdevicenum", sdhdevicenum, 0);
			n.param("dsadevicestring", dsadevicestring, std::string("/dev/ttyS0"));
			n.param("dsadevicenum", dsadevicenum, 0);

			// pointer to sdh
			sdh = new SDH::cSDH(false, false, 0); //(_use_radians=false, bool _use_fahrenheit=false, int _debug_level=0)

			//topicSub_JointCommand = n.subscribe("joint_commands", 1, &NodeClass::topicCallback_JointCommand, this);

			// implementation of service servers
			srvServer_Init = n.advertiseService("Init", &NodeClass::srvCallback_Init, this);
            srvServer_Stop = n.advertiseService("Stop", &NodeClass::srvCallback_Stop, this);
            srvServer_SetOperationMode = n.advertiseService("SetOperationMode", &NodeClass::srvCallback_SetOperationMode, this);
            
            // get JointNames from parameter server
			ROS_INFO("getting JointNames from parameter server");
			if (n.hasParam("JointNames"))
			{
				n.getParam("JointNames", JointNames_param);
			}
			else
			{
				ROS_ERROR("Parameter JointNames not set");
			}
			DOF_HW = JointNames_param.size(); // DOFs of sdh, NOTE: hardware has 8 DOFs, but two cuppled ones; joints palm_finger11 and palm_finger21 are actuated synchronously
			JointNames.resize(DOF_HW);
			for (int i = 0; i<DOF_HW; i++ )
			{
				JointNames[i] = (std::string)JointNames_param[i];
			}
			std::cout << "JointNames = " << JointNames_param << std::endl;
			
			ROS_INFO("getting JointNamesAll from parameter server");
			if (n.hasParam("JointNamesAll"))
			{
				n.getParam("JointNamesAll", JointNamesAll_param);
			}
			else
			{
				ROS_ERROR("Parameter JointNamesAll not set");
			}
			DOF_ROS = JointNamesAll_param.size(); // DOFs of sdh, NOTE: hardware has 8 DOFs, but two cuppled ones; joints palm_finger11 and palm_finger21 are actuated synchronously
			JointNamesAll.resize(DOF_ROS);
			for (int i = 0; i<DOF_ROS; i++ )
			{
				JointNamesAll[i] = (std::string)JointNamesAll_param[i];
			}
			std::cout << "JointNamesAll = " << JointNamesAll_param << std::endl;
			
			axes_.resize(DOF_HW);
			for(int i=0; i<DOF_HW; i++)
			{
				axes_[i] = i;
			}
			ROS_INFO("DOF_HW = %d, DOF_ROS = %d",DOF_HW,DOF_ROS);
		}

		// Destructor
		~NodeClass() 
		{
			sdh->Close();
			delete sdh;
		}

		// topic callback functions 
/*
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
*/

		void executeCB(const cob_actions::JointCommandGoalConstPtr &goal)
		{
			if (isInitialized == true)
			{
				// stop sdh first when new goal arrived
				try
				{
					sdh->Stop();
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			
			    std::string operationMode;
			    n.getParam("OperationMode", operationMode);
			    if (operationMode == "position")
			    {
				    ROS_DEBUG("moving sdh in position mode");
				        	
			    	std::vector<double> targetAngles; // in degrees
			    	targetAngles.resize(DOF_HW);
			    	targetAngles[0] = goal->command.positions[3]*180.0/pi_; // joint_palm_finger11
			    	targetAngles[1] = goal->command.positions[7]*180.0/pi_; // joint_finger21_finger22
			    	targetAngles[2] = goal->command.positions[8]*180.0/pi_; // joint_finger22_finger23
			    	targetAngles[3] = goal->command.positions[1]*180.0/pi_; // joint_thumb1_thumb2
			    	targetAngles[4] = goal->command.positions[2]*180.0/pi_; // joint_thumb2_thumb3
			    	targetAngles[5] = goal->command.positions[4]*180.0/pi_; // joint_finger11_finger12
			    	targetAngles[6] = goal->command.positions[5]*180.0/pi_; // joint_finger12_finger13
			    	ROS_INFO("received new position goal");
			    	std::cout << targetAngles[0] << " , " << targetAngles[1] << " , " << targetAngles[2] << " , " << targetAngles[3] << " , " << targetAngles[4] << " , " << targetAngles[5] << " , " << targetAngles[6] << std::endl;
			    	try
					{
						sdh->SetAxisTargetAngle( axes_, targetAngles );
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
			    else if (operationMode == "velocity")
			    {
			    	ROS_DEBUG("moving sdh in velocity mode");
			        //sdh->MoveVel(goal->trajectory.points[0].velocities);
			        ROS_WARN("Moving in velocity mode currently disabled");
			    }
			    else if (operationMode == "effort")
			    {
			    	ROS_DEBUG("moving sdh in effort mode");
			        //sdh->MoveVel(goal->trajectory.points[0].velocities);
			        ROS_WARN("Moving in effort mode currently disabled");
			    }
			    else
			    {
			        ROS_ERROR("sdh neither in position nor in velocity nor in effort mode. OperationMode = [%s]", operationMode.c_str());
			    }
			}
			else
			{
				ROS_DEBUG("sdh not initialized");
			}
			
			//if(success)
			//{
			//	std::string res = "executed JointCommand";
			//	result_.data = res.c_str();
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				// set the action state to succeeded
				as_.setSucceeded(result_);
			//}

		}
		
/*		void executeCB(const cob_actions::JointTrajectoryGoalConstPtr &goal)
		{
			ROS_INFO("Received new goal trajectory with %d points",goal->trajectory.points.size());
			ROS_INFO("%d", goal->trajectory.points[0].positions.size());
			// saving goal into local variables
			traj = goal->trajectory;
			traj_point_nr = 0;
			traj_point = traj.points[traj_point_nr];
			
			// stoping sdh to prepare for new trajectory
			std::vector<double> VelZero;
			VelZero.resize(DOF_HW);
			sdh->SetAxisTargetVelocity(axes_,VelZero);

			// check that preempt has not been requested by the client
			if (as_.isPreemptRequested())
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
			}

			// set the action state to succeed			
			result_.result.data = "executing trajectory";
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
*/

		// service callback functions
		// function will be called when a service is querried
		bool srvCallback_Init(cob_srvs::Trigger::Request &req,
				cob_srvs::Trigger::Response &res )
		{
			ROS_INFO("Initializing sdh");

			if (isInitialized == false)
			{
				//TODO: read from parameter
				//int _net=0;
				unsigned long _baudrate=1000000;
				double _timeout=0.02;
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
				/*
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
				*/
			}
			else
			{
				ROS_ERROR("...powercubes already initialized...");		        
				res.success = 1;
				res.errorMessage.data = "powercubes already initialized";
			}
			
			return true;
		}

		bool srvCallback_Stop(cob_srvs::Trigger::Request &req,
                              cob_srvs::Trigger::Response &res )
        {
       	    ROS_INFO("Stopping sdh");
        	
        	// set current trajectory to be finished
			//traj_point_nr = traj.points.size();
        	
            // stopping all arm movements
            sdh->Stop();
           	ROS_INFO("Stopping sdh succesfull");
           	res.success = 0; // 0 = true, else = false
            return true;
        }

        bool srvCallback_SetOperationMode(cob_srvs::SetOperationMode::Request &req,
                                          cob_srvs::SetOperationMode::Response &res )
        {
        	ROS_INFO("Set operation mode to [%s]", req.operationMode.data.c_str());
            n.setParam("OperationMode", req.operationMode.data.c_str());
            res.success = 0; // 0 = true, else = false
            return true;
        }

		void publishJointState()
        {
        	ROS_DEBUG("updateJointState");
        	
			if (isInitialized == true)
			{
				std::vector<double> actualAngles;
				actualAngles = sdh->GetAxisActualAngle( axes_ );
				
				ROS_DEBUG("received %d angles from sdh",actualAngles.size());
				
		        // create joint_state message
		        sensor_msgs::JointState msg;
				msg.header.stamp = ros::Time::now();
				msg.name.resize(DOF_ROS);
				msg.position.resize(DOF_ROS);

				// set joint names and map them to angles TODO: don't know if assignment is correct
				msg.name = JointNamesAll;
				//std::cout << actualAngles[0] << " , " << actualAngles[1] << " , " << actualAngles[2] << " , " << actualAngles[3] << " , " << actualAngles[4] << " , " << actualAngles[5] << " , " << actualAngles[6] << std::endl;
				msg.position[0] = 0.0; // joint_palm_thumb1
				msg.position[1] = actualAngles[3]*pi_/180.0; // joint_thumb1_thumb2
				msg.position[2] = actualAngles[4]*pi_/180.0; // joint_thumb2_thumb3
				msg.position[3] = actualAngles[0]*pi_/180.0; // joint_palm_finger11
				msg.position[4] = actualAngles[5]*pi_/180.0; // joint_finger11_finger12
				msg.position[5] = actualAngles[6]*pi_/180.0; // joint_finger12_finger13
				msg.position[6] = actualAngles[0]*pi_/180.0; // joint_palm_finger21
				msg.position[7] = actualAngles[1]*pi_/180.0; // joint_finger21_finger22
				msg.position[8] = actualAngles[2]*pi_/180.0; // joint_finger22_finger23
		            
		        // publish message
		        topicPub_JointState.publish(msg); 
			}
		}
/*
		void publishJointState()
		{
			ROS_INFO("updateJointState");
			std::vector<double> actualAngles;

			if(isInitialized == true)
			{
				ROS_INFO("isInitialized = true");
				//get actual joint positions 
				try
				{
					actualAngles = sdh->GetAxisActualAngle( axes_ );
				}
				catch (SDH::cSDHLibraryException* e)
				{
					ROS_ERROR("An exception was caught: %s", e->what());
					delete e;
				}
			}
			else
			{
				ROS_INFO("isInitialized = true");
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
			msg.name = JointNames;
			msg.position = actualAngles;
			for (int i = 0; i<DOF; i++ )
			{
				msg.position[i] = actualAngles[i];
			}
			msg.position[DOF+1] = actualAngles[DOF];
			
			//publish the message
			topicPub_JointState.publish(msg);

			//ROS_INFO("published JointState 3");
		}
*/

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
		 
		 
/*		void updateSdhCommands()
		{
			if (isInitialized == true)
			{
			    std::string operationMode;
			    n.getParam("OperationMode", operationMode);
			    if (operationMode == "position")
			    {
				    ROS_DEBUG("moving sdh in position mode");
				    std::vector<SDH::cSDH::eAxisState> v = sdh->GetAxisActualState(axes_);
			    	//if (v[0] == 'eAS_IDLE')
			    	if (true)
			    	{
				    	feedback_.isMoving = false;
				    	
				    	ROS_DEBUG("next point is %d from %d",traj_point_nr,traj.points.size());
				    	
				    	if (traj_point_nr < traj.points.size())
				    	{
				    		// if sdh is not moving and not reached last point of trajectory, the send new target point
				    		ROS_INFO("...moving to trajectory point[%d]",traj_point_nr);
					    	traj_point = traj.points[traj_point_nr];
					    	
					    	std::vector<double> targetAngles;
					    	targetAngles.resize(DOF_HW);
					    	targetAngles[0] = traj_point.positions[3]; // joint_palm_finger11
					    	targetAngles[1] = traj_point.positions[7]; // joint_finger21_finger22
					    	targetAngles[2] = traj_point.positions[8]; // joint_finger22_finger23
					    	targetAngles[3] = traj_point.positions[1]; // joint_thumb1_thumb2
					    	targetAngles[4] = traj_point.positions[2]; // joint_thumb2_thumb3
					    	targetAngles[5] = traj_point.positions[4]; // joint_finger11_finger12
					    	targetAngles[6] = traj_point.positions[5]; // joint_finger12_finger13
					    	std::cout << targetAngles[0] << " , " << targetAngles[1] << " , " << targetAngles[2] << " , " << targetAngles[3] << " , " << targetAngles[4] << " , " << targetAngles[5] << " , " << targetAngles[6] << std::endl;
					    	try
							{
								sdh->SetAxisTargetAngle( axes_, targetAngles );
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
					    	
//					    	PCube->MoveJointSpaceSync(traj_point.positions);

				    		traj_point_nr++;
					    	feedback_.isMoving = true;
					    	feedback_.pointNr = traj_point_nr;
	    					as_.publishFeedback(feedback_);
					    }
					    else
					    {
					    	ROS_DEBUG("...reached end of trajectory");
					    }
					}
					else
					{
						ROS_DEBUG("...sdh moving to point[%d]",traj_point_nr);
					}
			    }
			    else if (operationMode == "velocity")
			    {
			    	ROS_DEBUG("moving sdh in velocity mode");
			        //sdh->MoveVel(goal->trajectory.points[0].velocities);
			        ROS_WARN("Moving in velocity mode currently disabled");
			    }
			    else if (operationMode == "effort")
			    {
			    	ROS_DEBUG("moving sdh in effort mode");
			        //sdh->MoveVel(goal->trajectory.points[0].velocities);
			        ROS_WARN("Moving in effort mode currently disabled");
			    }
			    else
			    {
			        ROS_ERROR("sdh neither in position nor in velocity nor in effort mode. OperationMode = [%s]", operationMode.c_str());
			    }
			}
			else
			{
				ROS_DEBUG("sdh not initialized");
			}
		}
*/
}; //NodeClass

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// initialize ROS, spezify name of node
	ros::init(argc, argv, "cob_sdh");
	ROS_INFO("...sdh node running...");

	NodeClass nodeClass("JointCommand");
	sleep(1);
	ros::Rate loop_rate(5); // Hz
	while(nodeClass.n.ok())
	{
		// publish JointState
		nodeClass.publishJointState();
		////nodeClass.updateTactileData();
		
        // update commands to sdh
        //nodeClass.updateSdhCommands();

        // read parameter
        std::string operationMode;
        nodeClass.n.getParam("OperationMode", operationMode);
        ROS_DEBUG("running with OperationMode [%s]", operationMode.c_str());

		// sleep and waiting for messages, callbacks    
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
