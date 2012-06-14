/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering  
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_driver
 * ROS package name: cob_base_velocity_smoother
 *  							
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *  		
 * Author: Florian Mirus, email:Florian.Mirus@ipa.fhg.de
 *
 * Date of creation: May 2012
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *  	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *  	 notice, this list of conditions and the following disclaimer in the
 *  	 documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Fraunhofer Institute for Manufacturing 
 *  	 Engineering and Automation (IPA) nor the names of its
 *  	 contributors may be used to endorse or promote products derived from
 *  	 this software without specific prior written permission.
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
#include <ros/ros.h>
#include <XmlRpc.h>
#include <pthread.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <deque>
#include <sstream>
#include <iostream>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>

using namespace std;

/****************************************************************
 * the ros navigation doesn't run very smoothly because acceleration is too high
 * --> cob has strong base motors and therefore reacts with shaking behavior 
 * (PR2 has much more meachnical damping)
 * solution: the additional node cob_base_velocity_smoother smooths the velocities
 * comming from ROS-navigation, by calculating the mean values of a certain number 
 * of past messages and limiting the acceleration under a given threshold. 
 * cob_base_velocity_smoother subsribes (input) and publishes (output) geometry_msgs::Twist.
 ****************************************************************/
class cob_base_velocity_smoother
{
private:
	//capacity for circular buffers (to be loaded from parameter server, otherwise set to default value 12)
	int buffer_capacity;
	//maximal time-delay in seconds for stored messages in Circular Buffer (to be loaded from parameter server, otherwise set to default value 4)
	double store_delay;
	//threshhold for maximal allowed acceleration (to be loaded from parameter server, otherwise set to default value 0.02)
	double thresh;
	//geometry message filled with zero values
	geometry_msgs::Twist zero_values;

public:
	
	//constructor
	cob_base_velocity_smoother();

	//create node handle
	ros::NodeHandle n;

	//circular buffers for velocity, acceleration and time
	boost::circular_buffer<geometry_msgs::Twist> cb;
	boost::circular_buffer<geometry_msgs::Twist> cb_out;
	boost::circular_buffer<ros::Time> cb_time;

	//ros publisher
	ros::Publisher pub;

	//callback function to subsribe to the geometry messages cmd_vel and publish to base_controller/command
	void geometryCallback(const geometry_msgs::Twist& cmd_vel);
	//function that updates the circular buffer after receiving a new geometry message
	void reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel);
	
	//boolean function that returns true if all messages stored in the circular buffer are older than store_delay,
	//false otherwise
	bool CircBuffOutOfDate(ros::Time now);

	//functions to calculate the mean values for each direction
	double meanValueX();
	double meanValueY();
	double meanValueZ();

	//function for the actual computation
	//calls the reviseCircBuff and the meanValue-functions and limits the acceleration under thresh
	//returns the resulting geometry message to be published to the base_controller
	geometry_msgs::Twist setOutput(geometry_msgs::Twist cmd_vel);

};

//constructor
cob_base_velocity_smoother::cob_base_velocity_smoother()
{
	
	//get parameters from parameter server if possible or take default values
	if(n.hasParam("circular_buffer_capacity"))
	{
		n.getParam("circular_buffer_capacity",buffer_capacity);
	}
	else
	{
		buffer_capacity = 12;
		ROS_WARN("Used default parameter for circular buffer capacity [12]");
 	}

	if(n.hasParam("maximal_time_delay"))
	{
		n.getParam("maximal_time_delay",store_delay);
	}
	else
	{
		store_delay = 4;
		ROS_WARN("Used default parameter for maximal time delay in seconds for saved messages [4]");
 	}

	if(n.hasParam("thresh_max_acc"))
	{
		n.getParam("thresh_max_acc",thresh);
	}

	else
	{
		thresh = 0.02;
		ROS_WARN("Used default parameter for maximal allowed acceleration in m per s [0.02]");
 	}

	//set a geometry message containing zero-values
	zero_values.linear.x=0;
	zero_values.linear.y=0;
	zero_values.linear.z=0;

	zero_values.angular.x=0;
	zero_values.angular.y=0;
	zero_values.angular.z=0;

	//initialize circular buffers
	cb.set_capacity(buffer_capacity);
	cb_out.set_capacity(buffer_capacity);
	cb_time.set_capacity(buffer_capacity);
	
	//set actual ros::Time
	ros::Time now=ros::Time::now();

	//fill circular buffer with zero values
	while(cb.full() == false){

		cb.push_front(zero_values);
		cb_time.push_front(now);

	}

	pub = n.advertise<geometry_msgs::Twist>("output", 1);
	
};

//returns true if all messages in cb are out of date in consideration of store_delay
bool cob_base_velocity_smoother::CircBuffOutOfDate(ros::Time now)
{
	bool result=true;

	long unsigned int count=0;

	while( (count < cb.size()) && (result == true) ){
		
		double delay=(now.toSec() - cb_time[count].toSec());

		if(delay < store_delay){
			result = false;
		}
		count++;
	}

	return result;

};

//functions to calculate the mean values for linear/x
double cob_base_velocity_smoother::meanValueX()
{
	double result = 0;
	long unsigned int size = cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result = result + cb[i].linear.x;

	}
	result = result / size;

	if(size > 1){
		double max = cb[0].linear.x;
		long unsigned int max_ind = 0;
		for(long unsigned int i=0; i<size; i++){

			if(abs(result-cb[i].linear.x) > abs(result-max)){
				max = cb[i].linear.x;
				max_ind = i;
			}

		}

		//calculate sum
		for(long unsigned int i=0; i<size; i++){
		
			if(i != max_ind){
				result = result + cb[i].linear.x;
			}
		}
		result = result / (size - 1);
	}
	
	return result;
	
};

//functions to calculate the mean values for linear/y
double cob_base_velocity_smoother::meanValueY()
{
	double result = 0;
	long unsigned int size = cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result = result + cb[i].linear.y;

	}
	result = result / size;

	if(size > 1){

		double max = cb[0].linear.y;
		long unsigned int max_ind = 0;
		for(long unsigned int i=0; i<size; i++){

			if(abs(result-cb[i].linear.y) > abs(result-max)){
				max = cb[i].linear.y;
				max_ind = i;
			}

		}

		//calculate sum
		for(long unsigned int i=0; i<size; i++){
		
			if(i != max_ind){
				result = result + cb[i].linear.y;
			}
		}
		result = result / (size - 1);
	}

	return result;
	
};

//functions to calculate the mean values for angular/z
double cob_base_velocity_smoother::meanValueZ()
{
	double result = 0;
	long unsigned int size = cb.size();

	//calculate sum
	for(long unsigned int i=0; i<size; i++){

		result = result + cb[i].angular.z;

	}
	result = result / size;

	if(size > 1){

		double max = cb[0].angular.z;
		long unsigned int max_ind = 0;
		for(long unsigned int i=0; i<size; i++){

			if(abs(result-cb[i].angular.z) > abs(result-max)){
				max = cb[i].angular.z;
				max_ind = i;
			}

		}

		//calculate sum
		for(long unsigned int i=0; i<size; i++){
		
			if(i != max_ind){
				result = result + cb[i].angular.z;
			}
		}
		result = result / (size - 1);

	}

	return result;
	
};

//function that updates the circular buffer after receiving a new geometry message
void cob_base_velocity_smoother::reviseCircBuff(ros::Time now, geometry_msgs::Twist cmd_vel)
{
	if(this->CircBuffOutOfDate(now) == true){

		//clear buffers
		cb.clear();
		cb_time.clear();

		//fill circular buffer with zero_values and time buffer with actual time-stamp
		while(cb.full() == false){

			cb.push_front(zero_values);
			cb_time.push_front(now);

		}

		//add new command velocity message to circular buffer
		cb.push_front(cmd_vel);
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(now);

	}
	else{
		//cout << "revise CircBuff: else-case!" << endl;
		double delay=(now.toSec() - cb_time.back().toSec());

		while( delay >= store_delay ){
			//remove out-dated messages
			cb.pop_back();
			cb_time.pop_back();
	
			delay=(now.toSec() - cb_time.back().toSec());
		}

		//add new command velocity message to circular buffer
		cb.push_front(cmd_vel);
		//add new timestamp for subscribed command velocity message
		cb_time.push_front(now);

	}
};


//function for the actual computation
//calls the reviseCircBuff and the meanValue-functions and limits the acceleration under thresh
//returns the resulting geometry message to be published to the base_controller
geometry_msgs::Twist cob_base_velocity_smoother::setOutput(geometry_msgs::Twist cmd_vel)
{
	geometry_msgs::Twist result = zero_values;
	
	//set actual ros::Time
	ros::Time now=ros::Time::now();
	//update the circular buffers
	this->reviseCircBuff(now, cmd_vel);

	//calculate the mean values for each direction
	result.linear.x = meanValueX();
	result.linear.y = meanValueY();
	result.angular.z = meanValueZ();

	//limit the acceleration under thresh
	// only if cob_base_velocity_smoother has published a message yet
	if( cb_out.size() > 1){
	
		//set delty velocity and acceleration values
		double deltaX = result.linear.x - cb_out.front().linear.x;
		double accX = deltaX / ( now.toSec() - cb_time.front().toSec() );

		double deltaY = result.linear.y - cb_out.front().linear.y;
		double accY = deltaY / ( now.toSec() - cb_time.front().toSec() );

		double deltaZ = result.angular.z - cb_out.front().linear.y;
		double accZ = deltaZ /  ( now.toSec() - cb_time.front().toSec() );

		if( abs(accX) > thresh){
			result.linear.x = cb_out.front().linear.x + ( thresh *  ( now.toSec() - cb_time.front().toSec() ) );
		}
		if( abs(accY) > thresh){
			result.linear.y = cb_out.front().linear.y + ( thresh *  ( now.toSec() - cb_time.front().toSec() ));
		}
		if( abs(accZ) > thresh){
			result.angular.z = cb_out.front().angular.z + ( thresh *  ( now.toSec() - cb_time.front().toSec() ) );
		}

		cb_out.push_front(result);
	}

	return result;

}

//callback function to subsribe to the geometry messages cmd_vel and publish to base_controller/command
void cob_base_velocity_smoother::geometryCallback(const geometry_msgs::Twist& cmd_vel)
{

	//ROS_INFO("%s","I heard something, so here's the callBack-Function!");
	//cout << "subscribed-message: " << cmd_vel << endl;
	
	//generate Output messages
	geometry_msgs::Twist result = this->setOutput(cmd_vel);

	//print result
	//cout << "result message: " << result << endl;

	//publish result
	pub.publish(result);

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "cob_base_velocity_smoother");
	
	cob_base_velocity_smoother my_cvi = cob_base_velocity_smoother();

	ros::Subscriber sub=my_cvi.n.subscribe("input", 1, &cob_base_velocity_smoother::geometryCallback, &my_cvi);
	
	ros::spin();

	return 0;
}

