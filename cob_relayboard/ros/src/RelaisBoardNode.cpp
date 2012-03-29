/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <RelaisBoardNode.h>



int RelaisBoardNode::init() 
{
	if (n.hasParam("ComPort"))
	{
		n.getParam("ComPort", sComPort);
		ROS_INFO("Loaded ComPort parameter from parameter server: %s",sComPort.c_str());
	}

	n.param("message_timeout", relayboard_timeout_, 2.0);
	n.param("requestRate", requestRate, 25.0);

	n.param("protocol_version", protocol_version_, 1);
    

        n.param("voltage_min", voltage_min_, 23.0);
	n.param("voltage_max", voltage_max_, 27.5);
	n.param("charge_nominal", charge_nominal_, 80.0);
	charge_nominal_ = charge_nominal_ * 360; //converts [Ah] to [As]
	n.param("voltage_nominal", voltage_nominal_, 24.0);

	current_voltage = 0;

	m_SerRelayBoard = new SerRelayBoard();
	readConfig(protocol_version_); //sets config params for the serrelayboard
	m_SerRelayBoard->init();
	ROS_INFO("Opened Relayboard at ComPort = %s", sComPort.c_str());

	n.getParam("drive1/CANId", motorCanIdent[0]);
	n.getParam("drive2/CANId", motorCanIdent[1]);
	n.getParam("drive1/joint_name", joint_names[0]);
	n.getParam("drive2/joint_name", joint_names[1]);
	//topics, which get published if the module is available
	n.getParam("hasMotorRight",activeModule[DRIVE1]);
	n.getParam("hasMotorLeft",activeModule[DRIVE2]);
	if(activeModule[DRIVE1] == 1 || activeModule[DRIVE2] == 1)
	{
		topicPub_drives = n.advertise<cob_relayboard::DriveStates>("/drive_states",1);
		topicSub_drives = n.subscribe("/cmd_drives",1,&RelaisBoardNode::getNewDriveStates, this);
	}
	n.getParam("hasIOBoard", activeModule[IO_BOARD]);
	n.getParam("hasLCDOut", hasLCDOut);
	if(hasLCDOut == 1) topicSub_lcdDisplay = n.subscribe("/srb_lcd_display",1,&RelaisBoardNode::getNewLCDOutput, this);

	if(activeModule[IO_BOARD] == 1)
	{
		topicSub_setDigOut = n.subscribe("/srb_io_set_dig_out",1,&RelaisBoardNode::getIOBoardDigOut, this);
		topicPub_ioDigIn = n.advertise<std_msgs::Int16>("/srb_io_dig_in",1);
		topicPub_ioDigOut = n.advertise<std_msgs::Int16>("/srb_io_dig_out",1);
		topicPub_analogIn = n.advertise<cob_relayboard::IOAnalogIn>("/srb_io_analog_in",1);

	}
	n.getParam("hasUSBoard", activeModule[US_BOARD]);
	if(activeModule[US_BOARD] == 1)
	{
		topicPub_usBoard = n.advertise<cob_relayboard::USBoard>("/srb_us_measurements",1);
		topicSub_startUSBoard = n.subscribe("/srb_start_us_board",1,&RelaisBoardNode::startUSBoard, this);
		topicSub_stopUSBoard = n.subscribe("/srb_stop_us_board",1,&RelaisBoardNode::stopUSBoard, this);

	}
	n.getParam("hasRadarBoard", activeModule[RADAR_BOARD]);
	if(activeModule[RADAR_BOARD] == 1) topicPub_radarBoard = n.advertise<cob_relayboard::RadarBoard>("/srb_radar_measurements",1);

	n.getParam("hasGyroBoard", activeModule[GYRO_BOARD]);
	if(activeModule[GYRO_BOARD] == 1)
	{
		topicPub_gyroBoard = n.advertise<cob_relayboard::GyroBoard>("/srb_gyro_measurements",1);
		topicSub_zeroGyro = n.subscribe("/srb_zero_gyro",1,&RelaisBoardNode::zeroGyro, this);
	}

	n.getParam("hasKeyPad", hasKeyPad);
	if(hasKeyPad == 1) topicPub_keypad = n.advertise<cob_relayboard::Keypad>("/srb_keypad",1);
	n.getParam("hasIRSensors", hasIRSensors);
	if(hasIRSensors == 1) topicPub_IRSensor = n.advertise<cob_relayboard::IRSensors>("/srb_ir_measurements",1);






	// Init member variable for EM State
	EM_stop_status_ = ST_EM_ACTIVE;
	duration_for_EM_free_ = ros::Duration(1);
	return 0;
}

void RelaisBoardNode::readConfig(int protocol_version_)
{
	std::string pathToConf = "";
	int iTypeLCD = protocol_version_;
	std::string sNumComPort; 	
	int hasMotorRight;
	int hasMotorLeft;
	int hasIOBoard;

	int hasUSBoard;
	int hasRadarBoard;
	int hasGyroBoard;
	double quickfix1;
	double quickfix2;
	DriveParam driveParamLeft;
	DriveParam driveParamRight;


	n.getParam("ComPort", sNumComPort);
	n.getParam("hasMotorRight", hasMotorRight);
	n.getParam("hasMotorLeft", hasMotorLeft);
	n.getParam("hasIOBoard", hasIOBoard);
	n.getParam("hasUSBoard", hasUSBoard);
	n.getParam("hasRadarBoard", hasRadarBoard);
	n.getParam("hasGyroBoard", hasGyroBoard);


	if(n.hasParam("drive1/quickFix")) n.getParam("drive1/quickFix", quickfix1);
	else quickfix1 = 1;
	if(n.hasParam("drive2/quickFix")) n.getParam("drive2/quickFix", quickfix2);
	else quickfix2 = 1;


	int iEncIncrPerRevMot;
	double dVelMeasFrqHz;
	double dGearRatio, dBeltRatio;
	int iSign;
	bool bHoming;
	double dHomePos, dHomeVel;
	int iHomeEvent, iHomeDigIn, iHomeTimeOut;
	double dVelMaxEncIncrS, dVelPModeEncIncrS;
	double dAccIncrS2, dDecIncrS2;
	int iCANId;
	// drive parameters
	n.getParam("drive1/EncIncrPerRevMot", iEncIncrPerRevMot);
	n.getParam("drive1/VelMeasFrqHz", dVelMeasFrqHz);
	n.getParam("drive1/BeltRatio", dBeltRatio);
	n.getParam("drive1/GearRatio", dGearRatio);
	n.getParam("drive1/Sign", iSign);
	n.getParam("drive1/Homing", bHoming);
	n.getParam("drive1/HomePos", dHomePos);
	n.getParam("drive1/HomeVel", dHomeVel);
	n.getParam("drive1/HomeEvent", iHomeEvent);
	n.getParam("drive1/HomeDigIn", iHomeDigIn);
	n.getParam("drive1/HomeTimeOut", iHomeTimeOut);
	n.getParam("drive1/VelMaxEncIncrS", dVelMaxEncIncrS);
	n.getParam("drive1/VelPModeEncIncrS", dVelPModeEncIncrS);
	n.getParam("drive1/AccIncrS", dAccIncrS2);
	n.getParam("drive1/DecIncrS", dDecIncrS2);
	n.getParam("drive1/CANId", iCANId);
	driveParamLeft.set(	0,
							iEncIncrPerRevMot,
							dVelMeasFrqHz,
							dBeltRatio, dGearRatio,
							iSign,
							bHoming, dHomePos, dHomeVel, iHomeEvent, iHomeDigIn, iHomeTimeOut,
							dVelMaxEncIncrS, dVelPModeEncIncrS,
							dAccIncrS2, dDecIncrS2,
							DriveParam::ENCODER_INCREMENTAL,
							iCANId,
							false, true );
						
	n.getParam("drive2/EncIncrPerRevMot", iEncIncrPerRevMot);
	n.getParam("drive2/VelMeasFrqHz", dVelMeasFrqHz);
	n.getParam("drive2/BeltRatio", dBeltRatio);
	n.getParam("drive2/GearRatio", dGearRatio);
	n.getParam("drive2/Sign", iSign);
	n.getParam("drive2/Homing", bHoming);
	n.getParam("drive2/HomePos", dHomePos);
	n.getParam("drive2/HomeVel", dHomeVel);
	n.getParam("drive2/HomeEvent", iHomeEvent);
	n.getParam("drive2/HomeDigIn", iHomeDigIn);
	n.getParam("drive2/HomeTimeOut", iHomeTimeOut);
	n.getParam("drive2/VelMaxEncIncrS", dVelMaxEncIncrS);
	n.getParam("drive2/VelPModeEncIncrS", dVelPModeEncIncrS);
	n.getParam("drive2/AccIncrS", dAccIncrS2);
	n.getParam("drive2/DecIncrS", dDecIncrS2);
	n.getParam("drive2/CANId", iCANId);
	driveParamRight.set(	1,
							iEncIncrPerRevMot,
							dVelMeasFrqHz,
							dBeltRatio, dGearRatio,
							iSign,
							bHoming, dHomePos, dHomeVel, iHomeEvent, iHomeDigIn, iHomeTimeOut,
							dVelMaxEncIncrS, dVelPModeEncIncrS,
							dAccIncrS2, dDecIncrS2,
							DriveParam::ENCODER_INCREMENTAL,
							iCANId,
							false, true );

	m_SerRelayBoard->readConfig(	iTypeLCD, pathToConf, sNumComPort, 	hasMotorRight, 
					hasMotorLeft, hasIOBoard, hasUSBoard, hasRadarBoard, hasGyroBoard, 
					quickfix1, quickfix2, driveParamLeft, driveParamRight
			);
};

int RelaisBoardNode::requestBoardStatus() {
	int ret;	
	
	// Request Status of RelayBoard 
	ret = m_SerRelayBoard->sendRequest();

	if(ret != SerRelayBoard::NO_ERROR) {
		ROS_ERROR("Error in sending message to Relayboard over SerialIO, lost bytes during writing");
	}

	ret = m_SerRelayBoard->evalRxBuffer();
	if(ret==SerRelayBoard::NOT_INITIALIZED) {
		ROS_ERROR("Failed to read relayboard data over Serial, the device is not initialized");
		relayboard_online = false;
	} else if(ret==SerRelayBoard::NO_MESSAGES) {
		ROS_ERROR("For a long time, no messages from RelayBoard have been received, check com port!");
		if(time_last_message_received_.toSec() - ros::Time::now().toSec() > relayboard_timeout_) {relayboard_online = false;}
	} else if(ret==SerRelayBoard::TOO_LESS_BYTES_IN_QUEUE) {
		//ROS_ERROR("Relayboard: Too less bytes in queue");
	} else if(ret==SerRelayBoard::CHECKSUM_ERROR) {
		ROS_ERROR("A checksum error occurred while reading from relayboard data");
	} else if(ret==SerRelayBoard::NO_ERROR) {
		relayboard_online = true;
		relayboard_available = true;
		time_last_message_received_ = ros::Time::now();
	}

	return 0;
}

double RelaisBoardNode::getRequestRate()
{
	return requestRate;
}

//////////////
// RelaisBoard

void RelaisBoardNode::sendEmergencyStopStates()
{

	if(!relayboard_available) return;
	
	
	bool EM_signal;
	ros::Duration duration_since_EM_confirmed;
	cob_relayboard::EmergencyStopState EM_msg;

	// assign input (laser, button) specific EM state
	EM_msg.emergency_button_stop = m_SerRelayBoard->isEMStop();
	EM_msg.scanner_stop = m_SerRelayBoard->isScannerStop();

	// determine current EMStopState
	EM_signal = (EM_msg.emergency_button_stop || EM_msg.scanner_stop);

	switch (EM_stop_status_)
	{
		case ST_EM_FREE:
		{
			if (EM_signal == true)
			{
				ROS_ERROR("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			break;
		}
		case ST_EM_ACTIVE:
		{
			if (EM_signal == false)
			{
				ROS_INFO("Emergency stop was confirmed");
				EM_stop_status_ = EM_msg.EMCONFIRMED;
				time_of_EM_confirmed_ = ros::Time::now();
			}
			break;
		}
		case ST_EM_CONFIRMED:
		{
			if (EM_signal == true)
			{
				ROS_ERROR("Emergency stop was issued");
				EM_stop_status_ = EM_msg.EMSTOP;
			}
			else
			{
				duration_since_EM_confirmed = ros::Time::now() - time_of_EM_confirmed_;
				if( duration_since_EM_confirmed.toSec() > duration_for_EM_free_.toSec() )
				{
					ROS_INFO("Emergency stop released");
					EM_stop_status_ = EM_msg.EMFREE;
				}
			}
			break;
		}
	};

	
	EM_msg.emergency_state = EM_stop_status_;

	//publish EM-Stop-Active-messages, when connection to relayboard got cut
	if(relayboard_online == false) {
		EM_msg.emergency_state = EM_msg.EMSTOP;
	}
	topicPub_isEmergencyStop.publish(EM_msg);

        pr2_msgs::PowerBoardState pbs;
        pbs.header.stamp = ros::Time::now();
        if(EM_stop_status_ == ST_EM_FREE)
          pbs.run_stop = true;
        else
          pbs.run_stop = false;
        pbs.wireless_stop = true;
	pbs.input_voltage = current_voltage;
	topicPub_boardState.publish(pbs);
}


void RelaisBoardNode::sendAnalogIn()
{
	if(!relayboard_available) return;
	int analogIn[8];
	m_SerRelayBoard->getRelayBoardAnalogIn(analogIn);
	//temperatur
	cob_relayboard::Temperatur temp;
	temp.temperatur = analogIn[2];
	topicPub_temperatur.publish(temp);
	//battery
	pr2_msgs::PowerState bat;
	current_voltage = analogIn[1]/1000;
	bat.header.stamp = ros::Time::now();
	double percentage = ((analogIn[1] /*measured volts*/ / 1000.0) - voltage_min_) * 100 / (voltage_max_ - voltage_min_);
	/*dt_remaining = (i*dt)_nominal * v_nominal * percentage_remaining / (v_meassured * i_meassured)*/
	bat.relative_capacity = percentage;
	/* charging rate: analogIn[0];*/
	topicPub_batVoltage.publish(bat);
	//keypad
	if(hasKeyPad == 1)
	{
		cob_relayboard::Keypad pad;
		int mask = 1;
		for(int i = 0; i<4; i++)
		{
			if((analogIn[3] & mask) != 0)
			{
				pad.button[i] = true;
			} else {
				pad.button[i] = false;
			} 
			mask = mask << 1;
		}
		topicPub_keypad.publish(pad);
	}
	if(hasIRSensors == 1)
	{
		cob_relayboard::IRSensors irmsg;
		for(int i=0; i<4; i++) irmsg.measurement[i] = analogIn[4+i];
		topicPub_IRSensor.publish(irmsg);
	}
}

//////////////
// motorCtrl

void RelaisBoardNode::sendDriveStates()
{
	if(!relayboard_available) return;
	cob_relayboard::DriveStates state;
	for(int i = 0; i<2; i++)  state.joint_names[i] = joint_names[i];
	int temp;
	if(activeModule[DRIVE1] == 1 && activeModule[DRIVE2] == 1)
	{	
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[0],&(state.angularPosition[0]), &(state.angularVelocity[0]));
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[1],&(state.angularPosition[1]), &(state.angularVelocity[1]));
		m_SerRelayBoard->getStatus(motorCanIdent[0], &(state.motorState[0]), &temp);
		m_SerRelayBoard->getStatus(motorCanIdent[1], &(state.motorState[1]), &temp);
		topicPub_drives.publish(state);
	} 
	else if (activeModule[DRIVE1] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[0],&(state.angularPosition[0]), &(state.angularVelocity[0]));
		m_SerRelayBoard->getStatus(motorCanIdent[0], &(state.motorState[0]), &temp);
		topicPub_drives.publish(state);
	} 
	else if (activeModule[DRIVE2] == 1)
	{
		m_SerRelayBoard->getWheelPosVel(motorCanIdent[1],&(state.angularPosition[1]), &(state.angularVelocity[1]));
		m_SerRelayBoard->getStatus(motorCanIdent[1], &(state.motorState[1]), &temp);
		topicPub_drives.publish(state);
	}
}

void RelaisBoardNode::getNewDriveStates(const cob_relayboard::DriveCommands& driveCommands)
{
	//ROS_INFO("received drive command: %f   %f",driveCommands.angularVelocity[0],driveCommands.angularVelocity[1]);
	if(!relayboard_available) return;
	if(driveCommands.driveActive[0]){
		//TODO: test disableBrake:
		m_SerRelayBoard->disableBrake(motorCanIdent[0],driveCommands.disableBrake[0]);
		m_SerRelayBoard->setWheelVel(motorCanIdent[0], driveCommands.angularVelocity[0], driveCommands.quickStop[0]);
	}
	if(driveCommands.driveActive[1]){
		//TODO: test disableBrake:
		m_SerRelayBoard->disableBrake(motorCanIdent[1],driveCommands.disableBrake[1]);
		m_SerRelayBoard->setWheelVel(motorCanIdent[1], driveCommands.angularVelocity[1], driveCommands.quickStop[1]);
	}
}

//////////////
// GyroBoard

void RelaisBoardNode::sendGyroBoard()
{
	if(!relayboard_available || activeModule[GYRO_BOARD] != 1) return;
	cob_relayboard::GyroBoard gyro;
	m_SerRelayBoard->getGyroBoardAngBoost(&(gyro.orientation),gyro.acceleration);
	topicPub_gyroBoard.publish(gyro);
}

void RelaisBoardNode::zeroGyro(const std_msgs::Bool& b)
{
	if(!relayboard_available || activeModule[GYRO_BOARD] != 1) return;
	m_SerRelayBoard->zeroGyro(b.data);
}

//////////////
// radarBoard

void RelaisBoardNode::sendRadarBoard()
{
	if(!relayboard_available || activeModule[RADAR_BOARD] != 1) return;
	cob_relayboard::RadarBoard radar;
	double radarState[4];
	m_SerRelayBoard->getRadarBoardData(radarState);
	for(int i=0; i<3;i++)
	{
		radar.velocity[i] = radarState[i];
	}
	radar.state = radarState[3];
	topicPub_radarBoard.publish(radar);
}

//////////////
// usBoard

void RelaisBoardNode::sendUSBoard()
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	int usSensors[8];
	int usAnalog[4];
	cob_relayboard::USBoard usBoard;
	m_SerRelayBoard->getUSBoardData1To8(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i] = usSensors[i];
	m_SerRelayBoard->getUSBoardData9To16(usSensors);
	for(int i=0; i<8; i++) usBoard.sensor[i+8] = usSensors[i];
	m_SerRelayBoard->getUSBoardAnalogIn(usAnalog);
	for(int i=0; i<4; i++) usBoard.analog[i] = usAnalog[i];	
	topicPub_usBoard.publish(usBoard);
}

void RelaisBoardNode::startUSBoard(const std_msgs::Int16& configuration)
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	m_SerRelayBoard->startUS(configuration.data);
}

void RelaisBoardNode::stopUSBoard(const std_msgs::Empty& empty)
{
	if(!relayboard_available || activeModule[US_BOARD] != 1) return;
	m_SerRelayBoard->stopUS();
}

//////////////
// ioBoard

void RelaisBoardNode::getNewLCDOutput(const cob_relayboard::LCDOutput& msg) 
{
	if(!relayboard_available || hasLCDOut != 1) return;
	m_SerRelayBoard->writeIOBoardLCD(0,0,msg.msg_line);

}

void RelaisBoardNode::getIOBoardDigOut(const cob_relayboard::IOOut& setOut)
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
	m_SerRelayBoard->setIOBoardDigOut(setOut.channel, setOut.active);
}

void RelaisBoardNode::sendIOBoardDigIn()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
 	std_msgs::Int16 i;
	i.data = m_SerRelayBoard->getIOBoardDigIn();
	topicPub_ioDigIn.publish(i);

}

void RelaisBoardNode::sendIOBoardDigOut()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
 	std_msgs::Int16 i;
	i.data = m_SerRelayBoard->getIOBoardDigOut();
	topicPub_ioDigOut.publish(i);
}

void RelaisBoardNode::sendIOBoardAnalogIn()
{
	if(!relayboard_available || activeModule[IO_BOARD] != 1) return;
	int analogIn[8];
	cob_relayboard::IOAnalogIn in;
	m_SerRelayBoard->getIOBoardAnalogIn(analogIn);
	for(int i=0;i <8; i++) in.input[i] = analogIn[i];
	topicPub_analogIn.publish(in);
}
