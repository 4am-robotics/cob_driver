#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <brics_actuator/JointVelocities.h>
#include <cob_trajectory_controller/genericArmCtrl.h>


#define HZ 100

class cob_trajectory_controller
{
private:
    ros::NodeHandle n_;
    ros::Publisher joint_pos_pub_;
    ros::Publisher joint_vel_pub_;
    ros::Subscriber controller_state_;
    actionlib::SimpleActionServer<pr2_controllers_msgs::JointTrajectoryAction> as_;
    std::string action_name_;
    bool executing_;
    genericArmCtrl* traj_generator_;
    trajectory_msgs::JointTrajectory traj_;
    std::vector<double> q_current, startposition_, joint_distance_;

public:
    cob_trajectory_controller():as_(n_, "joint_trajectory_action", boost::bind(&cob_trajectory_controller::executeTrajectory, this, _1), true),
    action_name_("joint_trajectory_action")
    {
        joint_pos_pub_ = n_.advertise<sensor_msgs::JointState>("target_joint_pos", 1);
		joint_vel_pub_ = n_.advertise<brics_actuator::JointVelocities>("command_vel", 1);
        controller_state_ = n_.subscribe("state", 1, &cob_trajectory_controller::state_callback, this);
        executing_ = false;
		q_current.resize(7);
		traj_generator_ = new genericArmCtrl(7);
	}

  void state_callback(const pr2_controllers_msgs::JointTrajectoryControllerStatePtr& message)
  {
    std::vector<double> positions = message->actual.positions;
    for(unsigned int i = 0; i < positions.size(); i++)
    {
      q_current[i] = positions[i];
    }
  }
  void executeTrajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
  {
        ROS_INFO("Received new goal trajectory with %d points",goal->trajectory.points.size());
        if(!executing_)
        {
            traj_ = goal->trajectory;
            if(traj_.points.size() == 1)
            {
            	traj_generator_->moveThetas(traj_.points[0].positions, q_current);
            }
            else
            {
            	//Insert the current point as first point of trajectory, then generate SPLINE trajectory
            	trajectory_msgs::JointTrajectoryPoint p;
            	p.positions.resize(7);
            	p.velocities.resize(7);
            	p.accelerations.resize(7);
            	for(unsigned int i = 0; i<7; i++)
            	{
            		p.positions.at(i) = q_current.at(i);
            		p.velocities.at(i) = 0.0;
            		p.accelerations.at(i) = 0.0;
            	}
            	std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it;
            	it = traj_.points.begin();
            	traj_.points.insert(it,p);
            	traj_generator_->moveTrajectory(traj_, q_current);
            }
            executing_ = true;
            startposition_ = q_current;

            //TODO: std::cout << "Trajectory time: " << traj_end_time_ << " Trajectory step: " << traj_step_ << "\n";
		    }
	    else //suspend current movement and start new one
	    {
	   
	    }
		while(executing_)
		{
			sleep(1);
		}
		as_.setSucceeded();
    }
    
    void run()
    {
        if(executing_)
        {
        	std::vector<double> des_vel;
        	if(traj_generator_->step(q_current, des_vel))
        	{
        		if(!traj_generator_->isMoving) //Finished trajectory
        		{
        			executing_ = false;
        		}
				brics_actuator::JointVelocities target_joint_vel;
				target_joint_vel.velocities.resize(7);
				for(unsigned int i=0; i<7; i++)
				{
					std::stringstream joint_name;
					joint_name << "arm_" << (i+1) << "_joint";
					target_joint_vel.velocities[i].joint_uri = joint_name.str();
					target_joint_vel.velocities[i].unit = "rad";
					target_joint_vel.velocities[i].value = des_vel.at(i);

				}

				//send everything
				joint_vel_pub_.publish(target_joint_vel);
        	}
        	else
        	{
        		ROS_INFO("An controller error occured!");
        		executing_ = false;
        	}
        }
		else
		{	//WATCHDOG TODO: don't always send
			sensor_msgs::JointState target_joint_position;
			target_joint_position.position.resize(7);
			brics_actuator::JointVelocities target_joint_vel;
			target_joint_vel.velocities.resize(7);
			for (unsigned int i = 0; i < 7; i += 1)
			{
				std::stringstream joint_name;
				joint_name << "arm_" << (i+1) << "_joint";
				target_joint_vel.velocities[i].joint_uri = joint_name.str();
						target_joint_position.position[i] = 0;
				target_joint_vel.velocities[i].unit = "rad";
				target_joint_vel.velocities[i].value = 0;
					}
				joint_vel_pub_.publish(target_joint_vel);
				joint_pos_pub_.publish(target_joint_position);
			}
        
    }
    
};



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "cob_trajectory_controller");
    cob_trajectory_controller tm;
    ros::Rate loop_rate(HZ);
    while (ros::ok())
    {
        tm.run();
        ros::spinOnce();
        loop_rate.sleep();
    }  
	
}






