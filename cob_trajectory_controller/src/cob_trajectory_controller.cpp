#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/server/simple_action_server.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <brics_actuator/JointVelocities.h>


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
    trajectory_msgs::JointTrajectory traj_;
    double traj_time_;
    int current_point_;
    std::vector<double> q_current, startposition_, joint_distance_;
	double traj_end_time_;
	double traj_step_;

public:
    cob_trajectory_controller():as_(n_, "joint_trajectory_action", boost::bind(&cob_trajectory_controller::executeTrajectory, this, _1), true),
    action_name_("joint_trajectory_action")
    {
        joint_pos_pub_ = n_.advertise<sensor_msgs::JointState>("target_joint_pos", 1);
		joint_vel_pub_ = n_.advertise<brics_actuator::JointVelocities>("command_vel", 1);
        controller_state_ = n_.subscribe("state", 1, &cob_trajectory_controller::state_callback, this);
        traj_time_ = 0.0;
        current_point_ = 0;
        executing_ = false;
		q_current.resize(7);
		traj_end_time_ = 0;
		traj_step_ = 0;
	}

  void state_callback(const pr2_controllers_msgs::JointTrajectoryControllerStatePtr& message)
  {
    std::vector<double> positions = message->actual.positions;
    //std::cout << "Got Position: ";
    for(unsigned int i = 0; i < positions.size(); i++)
    {
      //std::cout << positions[i] << ", ";
      q_current[i] = positions[i];
    }
    //std::cout << "\n";
  }
  void executeTrajectory(const pr2_controllers_msgs::JointTrajectoryGoalConstPtr &goal)
  {
        ROS_INFO("Received new goal trajectory with %d points",goal->trajectory.points.size());
        if(!executing_)
        {
            traj_ = goal->trajectory;
            traj_time_ = 0.0;
            current_point_ = 0;
            executing_ = true;
            startposition_ = q_current;
			joint_distance_.resize(7);
			std::cout << "Goal: ";
			for(unsigned int i=0; i<7;i++)
			{
				std::cout << traj_.points[0].positions[i] << ", ";
		    }
			std::cout << "\nTraveling: ";
			double max_dist = 0;
			for(unsigned int i=0; i<7;i++)
			{
				joint_distance_[i] = traj_.points[traj_.points.size()-1].positions[i] - startposition_[i];
				std::cout << joint_distance_[i] << ", ";
				if(fabs(joint_distance_[i]) > max_dist)
					max_dist = fabs(joint_distance_[i]);
			}
			std::cout << "\n";

			traj_end_time_ = max_dist / 0.1;			
			traj_step_ = traj_end_time_ / traj_.points.size();
			std::cout << "Trajectory time: " << traj_end_time_ << " Trajectory step: " << traj_step_ << "\n";
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
            if(traj_time_ >= (current_point_ * traj_step_)-0.04)
            {
                if(current_point_ >= traj_.points.size()-5)
                {
                    ROS_INFO("Trajecory finished");
                    executing_ = false;
                    return;
                }
                else
                {
					ROS_INFO("Next Point");
                    current_point_ +=4;
					return;
                }
            }
	    	//calculate delta time
            double delta_time = 0.0;
            if(current_point_ != 0)
                delta_time = current_point_ * traj_step_ - traj_.points[current_point_-1].time_from_start.toSec();
            else
            	delta_time = current_point_ * traj_step_;
			double time_left = (current_point_ * traj_step_) - traj_time_;
			/*std::cout << "Goal: ";
			for(unsigned int i=0; i<7;i++)
			{
				std::cout << traj_.points[current_point_].positions[i] - q_current[i] << ", ";
		    }
			std::cout << "\nGoalVel: ";
			for(unsigned int i=0; i<7;i++)
			{
				std::cout << (traj_.points[current_point_].positions[i] - q_current[i])/time_left << ", ";
		    }
			std::cout << "\nTime from start: " << (traj_.points[current_point_].time_from_start.toSec()*20) << " Time left: " << time_left << "\n";*/

            //calculate current intermediate joint position
            sensor_msgs::JointState target_joint_position;
	    	target_joint_position.position.resize(7);
            for (unsigned int i = 0; i < traj_.points[current_point_].positions.size(); i += 1)
            {
                //position = current_time_inbetween * distance_to_travel/overall_time_inbetween
                target_joint_position.position[i] = (traj_time_ + 1./HZ) * (traj_.points[current_point_].positions[i] - startposition_[i])/delta_time;
                if(current_point_ != 0)
                {
                   target_joint_position.position[i] = (traj_time_ - traj_.points[current_point_-1].time_from_start.toSec() + 1./HZ) * (traj_.points[current_point_].positions[i] - traj_.points[current_point_-1].positions[i])/delta_time; 
                }
            }
			//calculate vel out of error error
			brics_actuator::JointVelocities target_joint_vel;
			target_joint_vel.velocities.resize(7);
			for(unsigned int i=0; i<7; i++)
			{
				std::stringstream joint_name;
				joint_name << "arm_" << (i+1) << "_joint";
				target_joint_vel.velocities[i].joint_uri = joint_name.str();
				target_joint_vel.velocities[i].unit = "rad";
				target_joint_vel.velocities[i].value = (traj_.points[current_point_].positions[i] - q_current[i])/(time_left);
				if(target_joint_vel.velocities[i].value >= 0.8)
				{
					std::cout << "Treshold " << i << "th joint: " << target_joint_vel.velocities[i].value << "\n";
					target_joint_vel.velocities[i].value = 0.0;
					ROS_INFO("Trajecory aborted");
                    executing_ = false;
                    return;
				}
				if(target_joint_vel.velocities[i].value <= -0.8)
				{
					std::cout << "Treshold " << i << "th joint: " << target_joint_vel.velocities[i].value << "\n";
					target_joint_vel.velocities[i].value = 0.0;
					ROS_INFO("Trajecory aborted");
                    executing_ = false;
                    return;
				}
				
			}

            //send everything
            joint_pos_pub_.publish(target_joint_position);
            joint_vel_pub_.publish(target_joint_vel);
            traj_time_ += 1./HZ;
        }
	else
	{
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






