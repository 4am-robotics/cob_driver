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

// general includes
#include <math.h>

// Headers provided by other cob-packages
#include <cob_generic_can/CanESD.h>
#include <cob_generic_can/CANPeakSys2PCI.h>
#include <cob_generic_can/CANPeakSysDongle.h>
#include <cob_generic_can/CANPeakSysUSB.h>
#include <cob_base_drive_chain/NeoCtrlPltfMpo500.h>
#include <sstream>


//-----------------------------------------------

NeoCtrlPltfMpo500::NeoCtrlPltfMpo500(ros::NodeHandle* node)
{	
	n = node;
	n->getParam("numberOfMotors", m_iNumMotors);
	m_pCanCtrl = NULL;

	// ------------- init hardware-specific vectors and set default values
	m_vpMotor.resize(m_iNumMotors);

	m_GearMotDrive = new GearMotorParamType[m_iNumMotors];
	for(int i=0; i<m_iNumMotors; i++)
	{
		m_vpMotor[i] = NULL;
	}
	m_viMotorID.resize(m_iNumMotors);
}

//-----------------------------------------------
NeoCtrlPltfMpo500::~NeoCtrlPltfMpo500()
{
	delete[] m_GearMotDrive;
	if (m_pCanCtrl != NULL)
	{
		delete m_pCanCtrl;
	}

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		if (m_vpMotor[i] != NULL)
		{
			delete m_vpMotor[i];
		}
	}

}

//-----------------------------------------------
void NeoCtrlPltfMpo500::readConfiguration()
{

	int iTypeCan = 0;
	int iMaxMessages = 0;

	DriveParam DriveParamDriveMotor[m_iNumMotors];
	

	// read Configuration of the Can-Network (CanCtrl.ini)
	n->getParam("can", iTypeCan);
	//can settings:
	int iBaudrateVal;
	std::string sCanDevice;
	n->getParam("BaudrateVal", iBaudrateVal);
	if (iTypeCan == 0)
	{	
 		if( n->hasParam("devicePath") ){
			n->getParam("devicePath", sCanDevice);
			ROS_INFO("set devicepath from config: %s %i",sCanDevice.c_str(), iBaudrateVal);
			m_pCanCtrl = new CANPeakSysDongle(iBaudrateVal, sCanDevice);
			
		}
		else m_pCanCtrl = new CANPeakSysDongle(iBaudrateVal);
		std::cout << "Uses CAN-Peak-Systems dongle" << std::endl;
	}
	else if (iTypeCan == 1)
	{
 		if( n->hasParam("devicePath") ){
			n->getParam("devicePath", sCanDevice);
			m_pCanCtrl = new CANPeakSysUSB(iBaudrateVal, sCanDevice);
		}
		else m_pCanCtrl = new CANPeakSysUSB(iBaudrateVal);
		std::cout << "Uses CAN-Peak-Systems USB" << std::endl;
	}
	else if (iTypeCan == 2)
	{
 		if( n->hasParam("devicePath") ){
			n->getParam("devicePath", sCanDevice);
			m_pCanCtrl = new CANPeakSys2PCI(iBaudrateVal, sCanDevice);
		}
		else m_pCanCtrl = new CANPeakSys2PCI(iBaudrateVal);
		std::cout << "Uses CAN-Peak-Systems PCI" << std::endl;
	}
	else if (iTypeCan == 3)
	{
 		if( n->hasParam("iNrNet") ){
			int iNrNet;
			n->getParam("iNrNet", iNrNet);
			m_pCanCtrl = new CanESD(iBaudrateVal, iNrNet);
		}
		else m_pCanCtrl = new CanESD(iBaudrateVal, 1);
		std::cout << "Uses CanESD" << std::endl;
	}

	std::string control_type[m_iNumMotors];
	// "Drive Motor Type1" drive parameters
	for(int i=0; i<m_iNumMotors; i++)
	{
		std::ostringstream stringstream;
		stringstream<<"drive"<<i<<"/";
		std::string pathName = stringstream.str();
		n->getParam(pathName + "control_type", control_type[i]);
		n->getParam(pathName + "EncIncrPerRevMot", m_GearMotDrive[i].iEncIncrPerRevMot);
		n->getParam(pathName + "VelMeasFrqHz", m_GearMotDrive[i].dVelMeasFrqHz);
		n->getParam(pathName + "BeltRatio", m_GearMotDrive[i].dBeltRatio);
		n->getParam(pathName + "GearRatio", m_GearMotDrive[i].dGearRatio);
		n->getParam(pathName + "Sign", m_GearMotDrive[i].iSign);
		n->getParam(pathName + "Homing", m_GearMotDrive[i].bHoming);
		n->getParam(pathName + "HomePos", m_GearMotDrive[i].dHomePos);
		n->getParam(pathName + "HomeVel", m_GearMotDrive[i].dHomeVel);
		n->getParam(pathName + "HomeEvent", m_GearMotDrive[i].iHomeEvent);
		n->getParam(pathName + "HomeDigIn", m_GearMotDrive[i].iHomeDigIn);
		n->getParam(pathName + "HomeTimeOut", m_GearMotDrive[i].iHomeTimeOut);
		n->getParam(pathName + "VelMaxEncIncrS", m_GearMotDrive[i].dVelMaxEncIncrS);
		n->getParam(pathName + "VelPModeEncIncrS", m_GearMotDrive[i].dVelPModeEncIncrS);
		n->getParam(pathName + "AccIncrS", m_GearMotDrive[i].dAccIncrS2);
		n->getParam(pathName + "DecIncrS", m_GearMotDrive[i].dDecIncrS2);
		n->getParam(pathName + "CANId", m_GearMotDrive[i].iCANId);

		m_viMotorID[i] = m_GearMotDrive[i].iCANId;

		DriveParamDriveMotor[i].set(	i,
						m_GearMotDrive[i].iEncIncrPerRevMot,
						m_GearMotDrive[i].dVelMeasFrqHz,
						m_GearMotDrive[i].dBeltRatio, m_GearMotDrive[i].dGearRatio,
						m_GearMotDrive[i].iSign,
						m_GearMotDrive[i].bHoming, m_GearMotDrive[i].dHomePos,
						m_GearMotDrive[i].dHomeVel, m_GearMotDrive[i].iHomeEvent,
						m_GearMotDrive[i].iHomeDigIn, m_GearMotDrive[i].iHomeTimeOut,
						m_GearMotDrive[i].dVelMaxEncIncrS, m_GearMotDrive[i].dVelPModeEncIncrS,
						m_GearMotDrive[i].dAccIncrS2, m_GearMotDrive[i].dDecIncrS2,
						DriveParam::ENCODER_INCREMENTAL,
						m_GearMotDrive[i].iCANId,
						false, true );



	}

	int rate;
	n->getParam("cycleRate", rate);
	double deltaT=1/( (double) rate);

	for(int i=0; i<m_iNumMotors; i++)
	{
		// Motor Harmonica
		ROS_INFO("Wheel1DriveMotor available, can_id: %i , cycle time: %f", m_GearMotDrive[i].iCANId, deltaT);
		CanDriveHarmonica* harmonica = new CanDriveHarmonica();
		harmonica->m_DriveParam = DriveParamDriveMotor[i];
		harmonica->setCANId(m_GearMotDrive[i].iCANId);
		harmonica->setCanItf(m_pCanCtrl);
		harmonica->setCycleTime(deltaT);
		harmonica->setTypeMotion(control_type[i]);
		m_vpMotor[i]=harmonica;
	}

	n->getParam("GenericBufferLen", iMaxMessages);

}

//-----------------------------------------------

bool NeoCtrlPltfMpo500::sendSynch()
{
        CanMsg msg;
        msg.m_iID  = 0x80;
        msg.m_iLen = 0;
        msg.set(0,0,0,0,0,0,0,0);
        m_pCanCtrl->transmitMsg(msg);
}

//-----------------------------------------------

int NeoCtrlPltfMpo500::evalCanBuffer()
{
	bool bRet;
//	char cBuf[200];
	
	m_Mutex.lock();

	// as long as there is something in the can buffer -> read out next message
	int count = 0;
	while(m_pCanCtrl->receiveMsg(&m_CanMsgRec) == true )
	{
		bRet = false;
		// check for every motor if message belongs to it
		for (unsigned int i = 0; i < m_vpMotor.size(); i++)
		{
			//ROS_DEBUG("evaluating can buffer");
			// if message belongs to this motor write data (Pos, Vel, ...) to internal member vars
			bRet |= m_vpMotor[i]->evalReceivedMsg(m_CanMsgRec);
		}


		if (bRet == false)
		{
			ROS_DEBUG("evalCanBuffer(): Received CAN_Message with unknown identifier or the message request was replied twice");
		}
	};


	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------

void NeoCtrlPltfMpo500::sendNetStartCanOpen()
{
	CanMsg msg;

	msg.m_iID  = 0;
	msg.m_iLen = 2;
	msg.set(1,0,0,0,0,0,0,0);
	m_pCanCtrl->transmitMsg(msg);

	usleep(100000);
}

//-----------------------------------------------

bool NeoCtrlPltfMpo500::initPltf()
{	
	// read Configuration parameters from yaml files
	readConfiguration();

	//start can open network
	sendNetStartCanOpen();
		
	//start motors
	for(int i=0; i < m_vpMotor.size(); i++)
	{
		if(m_vpMotor[i]->init(true))
		{
			ROS_INFO("starting motor: %i",i);
			if(m_vpMotor[i]->start())
			{
				ROS_INFO("motor started");
				m_vpMotor[i]->initWatchdog();
			}
		}
	}
	//homing:
	for(int i=0; i < m_vpMotor.size(); i++)
	{

		if(m_vpMotor[i]->m_DriveParam.getHoming() )
		{
			// start translational (index i+1) wheel synchronously
			// with homing drive (index i)

			//TODO: i+1
			m_vpMotor[i+1]->setWheelVel(m_vpMotor[i+1]->m_DriveParam.getSign() * 0.2, false, true);

			m_vpMotor[i]->prepareHoming();
			m_vpMotor[i]->initHoming();
			m_vpMotor[i]->execHoming();
			m_vpMotor[i]->isHomingFinished();
			
			// stop translational wheel
			m_vpMotor[i+1]->setWheelVel(0.0, false, true);
			
			usleep(50000);
			const double pi_ = 3.14159265;
			m_vpMotor[i]->setModuloCount(-pi_, pi_);
		}

	}

	return true;
}

//-----------------------------------------------
bool NeoCtrlPltfMpo500::resetPltf()
{
	bool bRetMotor = true;
	bool bRet = true;
	
	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		bRetMotor = m_vpMotor[i]->start();
		if (bRetMotor == false)
		{
			ROS_DEBUG("Resetting of Motor %i failed", i);
		}

		bRet &= bRetMotor;
	}
	return(bRet);
}

//-----------------------------------------------
bool NeoCtrlPltfMpo500::shutdownPltf()
{

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		m_vpMotor[i]->shutdown();
	}

	return true;
}

//-----------------------------------------------
bool NeoCtrlPltfMpo500::stopPltf()
{
        for(unsigned int i = 0; i < m_vpMotor.size(); i++)
        {
                m_vpMotor[i]->stop();
		setVelGearRadS(m_GearMotDrive[i].iCANId,0);
        }


};


//-----------------------------------------------
bool NeoCtrlPltfMpo500::isPltfError()
{
	bool errMotor[m_vpMotor.size()];
	bool overCurrent[m_vpMotor.size()];
	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		int tmp = 0;
		errMotor[i] = m_vpMotor[i]->isError(&tmp);
		if(errMotor[i])
		{
			return true;
		}
	}

	// Check communication
	double dWatchTime = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		dWatchTime = m_vpMotor[i]->getTimeToLastMsg();

		if( (dWatchTime > 1) && (m_bWatchdogErr == false) )
		{
			ROS_ERROR("timeout CAN motor %i", i );
			return true;
		}
	}

	return false;
}



//-----------------------------------------------
// Motor Controlers
//-----------------------------------------------


//-----------------------------------------------
int NeoCtrlPltfMpo500::setVelGearRadS(int iCanIdent, double dVelGearRadS)
{		
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			ROS_DEBUG("can send vel %f", dVelGearRadS);
			m_vpMotor[i]->setWheelVel(dVelGearRadS, false, true);
		}
	}
	
	m_Mutex.unlock();
	
	return 0;
}

//-----------------------------------------------
int NeoCtrlPltfMpo500::requestMotPosVel(int iCanIdent)
{
	m_Mutex.lock();

        for(unsigned int i = 0; i < m_vpMotor.size(); i++)
        {
                // check if Identifier fits to availlable hardware
                if(iCanIdent == m_viMotorID[i])
                {
                 	m_vpMotor[i]->requestPosVel(); //sends synch to all motors
                        ROS_DEBUG("can request vel measured, canid %i",  iCanIdent);
                }
        }	
	m_Mutex.unlock();

	return 0;
}

//-----------------------------------------------
void NeoCtrlPltfMpo500::requestDriveStatus()
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		m_vpMotor[i]->requestStatus();
	}

	m_Mutex.unlock();
}

//-----------------------------------------------
int NeoCtrlPltfMpo500::getGearPosVelRadS(int iCanIdent, double* pdAngleGearRad, double* pdVelGearRadS)
{

	// init default outputs
	*pdAngleGearRad = 0;
	*pdVelGearRadS = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			m_vpMotor[i]->getWheelPosVel(pdAngleGearRad, pdVelGearRadS);

			ROS_DEBUG("can received vel measured %f canid: %i ", *pdVelGearRadS, iCanIdent);
		}
	}
	
	return 0;
}

//-----------------------------------------------
int NeoCtrlPltfMpo500::getGearDeltaPosVelRadS(int iCanIdent, double* pdAngleGearRad,  double* pdVelGearRadS)
{

	// init default outputs
	*pdAngleGearRad = 0;
	*pdVelGearRadS = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			//TODO : m_vpMotor[i]->getWheelDeltaPosVel(pdAngleGearRad, pdVelGearRadS);
		}
	}

	return 0;
}

//-----------------------------------------------
void NeoCtrlPltfMpo500::getStatus(int iCanIdent, int* piStatus, int* piCurrentMeasPromille, int* piTempCel)
{
	// init default outputs
	*piStatus = 0;
	*piTempCel = 0;
	*piCurrentMeasPromille = 0;
	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			 m_vpMotor[i]->getStatus(piStatus, piCurrentMeasPromille, piTempCel);
		}
	}

}
//-----------------------------------------------
void NeoCtrlPltfMpo500::requestMotorTorque()
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		//TODO: m_vpMotor[i]->requestMotorTorque();
	}

	m_Mutex.unlock();
}	

//-----------------------------------------------
void NeoCtrlPltfMpo500::getMotorTorque(int iCanIdent, double* pdTorqueNm)
{
	// init default outputs
	*pdTorqueNm = 0;

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			//TODO: m_vpMotor[i]->getMotorTorque(pdTorqueNm);
		}
	}

}
//-----------------------------------------------
void NeoCtrlPltfMpo500::setMotorTorque(int iCanIdent, double dTorqueNm)
{
	m_Mutex.lock();

	for(unsigned int i = 0; i < m_vpMotor.size(); i++)
	{
		// check if Identifier fits to availlable hardware
		if(iCanIdent == m_viMotorID[i])
		{
			//TODO: m_vpMotor[i]->setMotorTorque(dTorqueNm);
		}
	}

	m_Mutex.unlock();
}
