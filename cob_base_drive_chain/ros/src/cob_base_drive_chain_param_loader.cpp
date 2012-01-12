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
 * ROS stack name: cob_intern
 * ROS package name: cob_base_drive_chain
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2012
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

#include <ros/ros.h>
#include <cob_base_drive_chain_param_loader.h>

cob_base_drive_chain_param_loader::cob_base_drive_chain_param_loader(ros::NodeHandle * node_handle) {
	n = node_handle;
}

PlatformParams cob_base_drive_chain_param_loader::get_platform_params() {
	read_configuration();
	
	return params;
};

void cob_base_drive_chain_param_loader::read_configuration() {
	//------------------------------------
	//Reading parameters of CAN device
	//------------------------------------
	if(!n->hasParam("can_device_path")) ROS_WARN("Used default parameter for can_device_path");
			n->param("can_device_path", params.can_device_params.sDevicePath, std::string("/dev/pcan2"));
	
	if(!n->hasParam("can_baudrate_val")) ROS_WARN("Used default parameter for can_baudrate_val");
			n->param("can_baudrate_val", params.can_device_params.iBaudrateVal, 0);
	
	//------------------------------------
	//Reading general parameters
	//------------------------------------
	if(!n->hasParam("number_of_wheels")) ROS_WARN("Used default parameter for number_of_wheels");
			n->param("number_of_wheels", params.num_wheels, 4);
	
	if(!n->hasParam("radius_wheels")) ROS_WARN("Used default parameter for radius_wheels");
			n->param("radius_wheels", params.platform_config.iRadiusWheelMM, 75);
	
	if(!n->hasParam("dist_steer_axis_to_drive_wheel")) ROS_WARN("Used default parameter for dist_steer_axis_to_drive_wheel");
			n->param("dist_steer_axis_to_drive_wheel", params.platform_config.iDistSteerAxisToDriveWheelMM, 22);
			
	if(!n->hasParam("can_comm_timeout")) ROS_WARN("Used default parameter for can_comm_timeout");
			n->param("can_comm_timeout", params.platform_config.dCanTimeout, 7.0);
			
	if(!n->hasParam("max_drive_rate")) ROS_WARN("Used default parameter for max_drive_rate");
			n->param("max_drive_rate", params.max_drive_rate_rad_s, 12.267);
			
	if(!n->hasParam("max_steer_rate")) ROS_WARN("Used default parameter for max_steer_rate");
			n->param("max_steer_rate", params.max_steer_rate_rad_s, 24.4);
			
	// 0: no motor
	// 1: motor of the type neo
	// 2: motor of the type ham
	params.platform_config.iHasWheel1DriveMotor = 0;
	params.platform_config.iHasWheel1SteerMotor = 0;
	params.platform_config.iHasWheel2DriveMotor = 0;
	params.platform_config.iHasWheel2SteerMotor = 0;
	params.platform_config.iHasWheel3DriveMotor = 0;
	params.platform_config.iHasWheel3SteerMotor = 0;
	params.platform_config.iHasWheel4DriveMotor = 0;
	params.platform_config.iHasWheel4SteerMotor = 0;

	if(params.num_wheels >= 1) {
		params.platform_config.iHasWheel1DriveMotor = 2;
		params.platform_config.iHasWheel1SteerMotor = 2;}
	if(params.num_wheels >= 2) {
		params.platform_config.iHasWheel2DriveMotor = 2;
		params.platform_config.iHasWheel2SteerMotor = 2;}
	if(params.num_wheels >= 3){
		params.platform_config.iHasWheel3DriveMotor = 2;
		params.platform_config.iHasWheel3SteerMotor = 2;}
	if(params.num_wheels == 4){
		params.platform_config.iHasWheel4DriveMotor = 2;
		params.platform_config.iHasWheel4SteerMotor = 2;}

	
	//------------------------------------
	//Reading parameters for each wheel
	//------------------------------------
	ros::NodeHandle * cur_joint_nh;
	
	params.drive_param_drive.assign(params.num_wheels, *(new GearMotorParamType));
	params.drive_param_steer.assign(params.num_wheels, *(new GearMotorParamType));
	params.steer_drive_coupling.assign(params.num_wheels, 0.0);
	params.wheel_neutral_pos.assign(params.num_wheels, 0.0);
	int can_id_temp;
	
	for(int i = 0; i<params.num_wheels; i++) {
	
	//Wheel#i, Drive
		if(i==0) cur_joint_nh = new ros::NodeHandle(*n, "caster_front_left/drive");
		else if (i==1) cur_joint_nh = new ros::NodeHandle(*n, "caster_back_left/drive");
		else if (i==2) cur_joint_nh = new ros::NodeHandle(*n, "caster_back_right/drive");
		else if (i==3) cur_joint_nh = new ros::NodeHandle(*n, "caster_front_right/drive");
		
		if(!cur_joint_nh->hasParam("SteerDriveCoupling")) ROS_WARN("Used default parameter for Wheel%d/drive SteerDriveCoupling", i+1);
			cur_joint_nh->param("SteerDriveCoupling", params.steer_drive_coupling.at(i), 0.0);
			
		if(!cur_joint_nh->hasParam("WheelNeutralPosition")) ROS_WARN("Used default parameter for Wheel%d/drive WheelNeutralPosition", i+1);
			cur_joint_nh->param("WheelNeutralPosition", params.wheel_neutral_pos.at(i), 0.0);
		
		if(!cur_joint_nh->hasParam("EncIncrPerRevMot")) ROS_WARN("Used default parameter for Wheel%d/drive EncIncrPerRevMot", i+1);
			cur_joint_nh->param("EncIncrPerRevMot", params.drive_param_drive.at(i).iEncIncrPerRevMot, 4096);
		if(!cur_joint_nh->hasParam("VelMeasFrqHz")) ROS_WARN("Used default parameter for Wheel%d/drive VelMeasFrqHz", i+1);
			cur_joint_nh->param("VelMeasFrqHz", params.drive_param_drive.at(i).dVelMeasFrqHz, 1.0);
		if(!cur_joint_nh->hasParam("BeltRatio")) ROS_WARN("Used default parameter for Wheel%d/drive BeltRatio", i+1);
			cur_joint_nh->param("BeltRatio", params.drive_param_drive.at(i).dBeltRatio, 2.0);
		if(!cur_joint_nh->hasParam("GearRatio")) ROS_WARN("Used default parameter for Wheel%d/drive GearRatio", i+1);
			cur_joint_nh->param("GearRatio", params.drive_param_drive.at(i).dGearRatio, 15.0);
		if(!cur_joint_nh->hasParam("Sign")) ROS_WARN("Used default parameter for Wheel%d/drive Sign", i+1);
			cur_joint_nh->param("Sign", params.drive_param_drive.at(i).iSign, 1);
		if(!cur_joint_nh->hasParam("VelMaxEncIncrS")) ROS_WARN("Used default parameter for Wheel%d/drive VelMaxEncIncrS", i+1);
			cur_joint_nh->param("VelMaxEncIncrS", params.drive_param_drive.at(i).dVelMaxEncIncrS, 240000.0);
		if(!cur_joint_nh->hasParam("AccIncrS")) ROS_WARN("Used default parameter for Wheel%d/drive AccIncrS", i+1);
			cur_joint_nh->param("AccIncrS", params.drive_param_drive.at(i).dAccIncrS2, 500000.0);
		if(!cur_joint_nh->hasParam("DecIncrS")) ROS_WARN("Used default parameter for Wheel%d/drive DecIncrS", i+1);
			cur_joint_nh->param("DecIncrS", params.drive_param_drive.at(i).dDecIncrS2, 500000.0);
		if(!cur_joint_nh->hasParam("EncOffsetIncr")) ROS_WARN("Used default parameter for Wheel%d/drive EncOffsetIncr", i+1);
			cur_joint_nh->param("EncOffsetIncr", params.drive_param_drive.at(i).iEncOffsetIncr, 0);
		if(!cur_joint_nh->hasParam("IsSteering")) ROS_WARN("Used default parameter for Wheel%d/drive IsSteering", i+1);
			cur_joint_nh->param("IsSteering", params.drive_param_drive.at(i).bIsSteer, false);
		if(!cur_joint_nh->hasParam("CurrentToTorque")) ROS_WARN("Used default parameter for Wheel%d/drive CurrentToTorque", i+1);
			cur_joint_nh->param("CurrentToTorque", params.drive_param_drive.at(i).dCurrentToTorque, 0.10065);
		if(!cur_joint_nh->hasParam("CurrMax")) ROS_WARN("Used default parameter for Wheel%d/drive CurrMax", i+1);
			cur_joint_nh->param("CurrMax", params.drive_param_drive.at(i).dCurrMax, 9.62);
		if(!cur_joint_nh->hasParam("HomingDigIn")) ROS_WARN("Used default parameter for Wheel%d/drive HomingDigIn", i+1);
			cur_joint_nh->param("HomingDigIn", params.drive_param_drive.at(i).iHomingDigIn, 9);
		//Read CAN-open-ids:
		if(!cur_joint_nh->hasParam("CanOpenID")) ROS_FATAL("Used default parameter for Wheel%d/drive CanOpenID", i+1);
			cur_joint_nh->param("CanOpenID", can_id_temp, 1);
			if(i==0) {
				params.canopen_ids.TxPDO1_W1Drive = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W1Drive = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W1Drive = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W1Drive = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W1Drive = 0x601 + can_id_temp -1;
			} else if(i==1) {
				params.canopen_ids.TxPDO1_W2Drive = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W2Drive = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W2Drive = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W2Drive = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W2Drive = 0x601 + can_id_temp -1;
			} else if(i==2) {
				params.canopen_ids.TxPDO1_W3Drive = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W3Drive = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W3Drive = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W3Drive = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W3Drive = 0x601 + can_id_temp -1;
			} else if(i==3) {
				params.canopen_ids.TxPDO1_W4Drive = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W4Drive = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W4Drive = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W4Drive = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W4Drive = 0x601 + can_id_temp -1;
			}
		
	
	//Wheel#i, Steer
		if(i==0) cur_joint_nh = new ros::NodeHandle(*n, "caster_front_left/steer");
		else if (i==1) cur_joint_nh = new ros::NodeHandle(*n, "caster_back_left/steer");
		else if (i==2) cur_joint_nh = new ros::NodeHandle(*n, "caster_back_right/steer");
		else if (i==3) cur_joint_nh = new ros::NodeHandle(*n, "caster_front_right/steer");
		
		if(!cur_joint_nh->hasParam("EncIncrPerRevMot")) ROS_WARN("Used default parameter for Wheel%d/steer EncIncrPerRevMot", i+1);
			cur_joint_nh->param("EncIncrPerRevMot", params.drive_param_drive.at(i).iEncIncrPerRevMot, 4096);
		if(!cur_joint_nh->hasParam("VelMeasFrqHz")) ROS_WARN("Used default parameter for Wheel%d/steer VelMeasFrqHz", i+1);
			cur_joint_nh->param("VelMeasFrqHz", params.drive_param_drive.at(i).dVelMeasFrqHz, 1.0);
		if(!cur_joint_nh->hasParam("BeltRatio")) ROS_WARN("Used default parameter for Wheel%d/steer BeltRatio", i+1);
			cur_joint_nh->param("BeltRatio", params.drive_param_drive.at(i).dBeltRatio, 1.0);
		if(!cur_joint_nh->hasParam("GearRatio")) ROS_WARN("Used default parameter for Wheel%d/steer GearRatio", i+1);
			cur_joint_nh->param("GearRatio", params.drive_param_drive.at(i).dGearRatio, 12.66666666);
		if(!cur_joint_nh->hasParam("Sign")) ROS_WARN("Used default parameter for Wheel%d/steer Sign", i+1);
			cur_joint_nh->param("Sign", params.drive_param_drive.at(i).iSign, 1);
		if(!cur_joint_nh->hasParam("VelMaxEncIncrS")) ROS_WARN("Used default parameter for Wheel%d/steer VelMaxEncIncrS", i+1);
			cur_joint_nh->param("VelMaxEncIncrS", params.drive_param_drive.at(i).dVelMaxEncIncrS, 240000.0);
		if(!cur_joint_nh->hasParam("AccIncrS")) ROS_WARN("Used default parameter for Wheel%d/steer AccIncrS", i+1);
			cur_joint_nh->param("AccIncrS", params.drive_param_drive.at(i).dAccIncrS2, 1000000.0);
		if(!cur_joint_nh->hasParam("DecIncrS")) ROS_WARN("Used default parameter for Wheel%d/steer DecIncrS", i+1);
			cur_joint_nh->param("DecIncrS", params.drive_param_drive.at(i).dDecIncrS2, 1000000.0);
		if(!cur_joint_nh->hasParam("EncOffsetIncr")) ROS_WARN("Used default parameter for Wheel%d/steer EncOffsetIncr", i+1);
			cur_joint_nh->param("EncOffsetIncr", params.drive_param_drive.at(i).iEncOffsetIncr, 0);
		if(!cur_joint_nh->hasParam("IsSteering")) ROS_WARN("Used default parameter for Wheel%d/steer IsSteering", i+1);
			cur_joint_nh->param("IsSteering", params.drive_param_drive.at(i).bIsSteer, true);
		if(!cur_joint_nh->hasParam("CurrentToTorque")) ROS_WARN("Used default parameter for Wheel%d/steer CurrentToTorque", i+1);
			cur_joint_nh->param("CurrentToTorque", params.drive_param_drive.at(i).dCurrentToTorque, 0.10065);
		if(!cur_joint_nh->hasParam("CurrMax")) ROS_WARN("Used default parameter for Wheel%d/steer CurrMax", i+1);
			cur_joint_nh->param("CurrMax", params.drive_param_drive.at(i).dCurrMax, 9.62);
		if(!cur_joint_nh->hasParam("HomingDigIn")) ROS_WARN("Used default parameter for Wheel%d/steer HomingDigIn", i+1);
			cur_joint_nh->param("HomingDigIn", params.drive_param_drive.at(i).iHomingDigIn, 9);
		//Read CAN-open-ids:
		if(!cur_joint_nh->hasParam("CanOpenID")) ROS_FATAL("Used default parameter for Wheel%d/steer CanOpenID", i+1);
			cur_joint_nh->param("CanOpenID", can_id_temp, 1);
			if(i==0) {
				params.canopen_ids.TxPDO1_W1Steer = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W1Steer = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W1Steer = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W1Steer = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W1Steer = 0x601 + can_id_temp -1;
			} else if(i==1) {
				params.canopen_ids.TxPDO1_W2Steer = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W2Steer = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W2Steer = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W2Steer = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W2Steer = 0x601 + can_id_temp -1;
			} else if(i==2) {
				params.canopen_ids.TxPDO1_W3Steer = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W3Steer = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W3Steer = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W3Steer = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W3Steer = 0x601 + can_id_temp -1;
			} else if(i==3) {
				params.canopen_ids.TxPDO1_W4Steer = 0x181 + can_id_temp -1;
				params.canopen_ids.TxPDO2_W4Steer = 0x281 + can_id_temp -1;
				params.canopen_ids.RxPDO2_W4Steer = 0x301 + can_id_temp -1;
				params.canopen_ids.TxSDO_W4Steer = 0x581 + can_id_temp -1;
				params.canopen_ids.RxSDO_W4Steer = 0x601 + can_id_temp -1;
			}
	}
	
	return;
}




