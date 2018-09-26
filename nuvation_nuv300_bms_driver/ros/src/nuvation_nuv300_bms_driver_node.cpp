/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <ros/ros.h>
#include <cob_msgs/PowerState.h>
#include <modbus/modbus.h>


class Nuv300BMS
{
public:
	Nuv300BMS(ros::NodeHandle& nh)
	: node_handle_(nh), sequence_counter_(0)
	{
		// params
		// todo: data logging

		// setup ros publisher
		power_state_pub_ = node_handle_.advertise<cob_msgs::PowerState>("power_state", 0);

		// connect to modbus
		modbus_t *mb;
		mb = modbus_new_tcp("192.168.1.21", 502);
		modbus_connect(mb);

		ros::Rate loop_rate(10);
		while(ros::ok())
		{
			// From: https://www.nuvationenergy.com/technical-resources
			// Communication Protocol Reference Guide: https://www.nuvationenergy.com/sites/default/files/Nuvation-Energy-Images/Technical-Resources/Nuvation-BMS-Communication-Protocol-Reference-Guide.pdf
			// Further specification: http://mesastandards.org/wp-content/uploads/2015/10/Energy-Storage-Information-Models_D3-2015-10-26-Update.xlsx
			//
			// Model	Block	Point Name		Address		Type		Unit	Scale Factor	Purpose
			// 801		Fixed	SoC				40081		uint16		%		SoC_SF			BMS State of Charge
			// 801		Fixed	SoC_SF			40092		sunssf								Scale Factor for SoC
			// 802		Fixed	Vol				40105		uint16		V		Vol_SF			External DC voltage of the battery system
			// 802		Fixed	Vol_SF			40113		sunssf								Scale Factor for Vol
			// 803		Fixed	BTotDCCur		40127		int16		A		BCurrent_SF		Total DC current of the battery system
			// 803		Fixed	BCurrent_SF		40132		sunssf								Scale Factor for BTotDCCur
			//?803		Repeat	StrCur			40137+Index	int16		A		BCurrent_SF		Current of a stack/string ?
			//
			// The term Index in the Repeating block addresses used in the above table refers to a calculation of
			// Index = Stack Index * Length of Repeating block. By definition, the 803 Repeating block is 16 Modbus registers in length.

			// Read 5 registers from the address 0
			uint16_t tab_reg[32];
			modbus_read_registers(mb, 0, 5, tab_reg);

			// publish PowerState message
			cob_msgs::PowerState msg;
			msg.header.seq = sequence_counter_++;
			msg.header.stamp = ros::Time::now();
			msg.voltage = 0;
			msg.current = 0;
			msg.charging = false;
			msg.remaining_capacity = 0.;
			msg.relative_remaining_capacity = 0.;
			power_state_pub_.publish(msg);

			// data logging

			ros::spinOnce();
			loop_rate.sleep();
		}

		modbus_close(mb);
		modbus_free(mb);
	}

protected:

	ros::NodeHandle node_handle_;

	ros::Publisher power_state_pub_;
	unsigned int sequence_counter_;
};

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "nuvation_nuv300_bms_driver_node");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of the BMS driver
	Nuv300BMS bms(nh);

	return (0);
}
