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
		// parameters
		ros::NodeHandle pnh("~");
		std::cout << "\n========== Nuv300BMS Parameters ==========\n";
		pnh.param("loop_rate", loop_rate_, 10.);
		std::cout << "loop_rate: " << loop_rate_ << std::endl;
		pnh.param("bms_ip_address", bms_ip_address_, std::string("192.168.1.21"));
		std::cout << "bms_ip_address: " << bms_ip_address_ << std::endl;
		pnh.param("bms_port", bms_port_, 502);
		std::cout << "bms_port: " << bms_port_ << std::endl;

		// setup ros publisher
		power_state_pub_ = node_handle_.advertise<cob_msgs::PowerState>("power_state", 0);

		// connect to modbus
		modbus_t *mb;
		mb = modbus_new_tcp(bms_ip_address_.c_str(), bms_port_);
		modbus_connect(mb);

		// receive the scale factors
		uint16_t tab_reg[32];
		if (modbus_read_registers(mb, 40092, 1, tab_reg) == -1)
			fprintf(stderr, "%s\n", modbus_strerror(errno));
		const double soc_sf = pow(10,(double)((int16_t*)tab_reg)[0]);
		if (modbus_read_registers(mb, 40113, 1, tab_reg) == -1)
			fprintf(stderr, "%s\n", modbus_strerror(errno));
		const double vol_sf = pow(10,(double)((int16_t*)tab_reg)[0]);
		if (modbus_read_registers(mb, 40132, 1, tab_reg) == -1)
			fprintf(stderr, "%s\n", modbus_strerror(errno));
		const double bcurrent_sf = pow(10,(double)((int16_t*)tab_reg)[0]);
		
		std::cout << "soc_sf=" << soc_sf << "   vol_sf=" << vol_sf << "   bcurrent_sf=" << bcurrent_sf << std::endl;

		ros::Rate loop_rate(loop_rate_);
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

			cob_msgs::PowerState msg;

			// read SoC
			modbus_read_registers(mb, 40081, 1, tab_reg);
			msg.relative_remaining_capacity = (double)tab_reg[0]*soc_sf;

			// read Vol
			modbus_read_registers(mb, 40105, 1, tab_reg);
			msg.voltage = (double)tab_reg[0]*vol_sf;

			// read BTotDCCur
			modbus_read_registers(mb, 40127, 1, tab_reg);
			msg.current = (double)((int16_t*)tab_reg)[0]*bcurrent_sf;
			
			// publish PowerState message
			msg.header.seq = sequence_counter_++;
			msg.header.stamp = ros::Time::now();
			msg.charging = (msg.current < 0. ? true : false);		// todo: check whether this assumption is correct
			msg.remaining_capacity = 0.;
			power_state_pub_.publish(msg);

			ros::spinOnce();
			loop_rate.sleep();
		}

		modbus_close(mb);
		modbus_free(mb);
	}

protected:

	ros::NodeHandle node_handle_;

	double loop_rate_;		// loop rate for publishing BMS data
	std::string bms_ip_address_;		// ip address of the BMS
	int bms_port_;				// network port to BMS

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
