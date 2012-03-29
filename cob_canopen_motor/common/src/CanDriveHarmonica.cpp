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


#include <stdlib.h>
#include <cob_canopen_motor/CanDriveHarmonica.h>

//#define LOG_ENABLED
//#define LOG_MSG_ENABLED

// measure velocity or torque data from amplifier
//#define MEASURE_VELOCITY

//-----------------------------------------------
CanDriveHarmonica::CanDriveHarmonica()
{
	// Parameter
	m_Param.iDivForRequestStatus = 10;
	m_Param.dCANTimeout = 2.0;

	// Variables
	m_pCanCtrl = NULL;

	m_iStatusCtrl = 0;
	m_dPosWheelMeasRad = 0;
	m_dLastPos = 0;
	m_dLastVel = 0;
	m_dAngleWheelRadMem  = 0;
	m_dVelWheelMeasRadS = 0;
	m_iCurrentMeasPromille = 0;
	critical_state = false;

	m_iMotorState = ST_PRE_INITIALIZED;
	m_iCommState = m_bCurrentLimitOn = false;

	last_pose_incr = 0;
	//
	IniFile iFile;

	m_Param.dCycleTime = 5;
/* TODO:
#ifdef APPLICATION_TYPE_PLATFORMCONTROL
	iFile.SetFileName ( "Platform/IniFiles/Platform.ini", "CanDriveHarmonica.cpp" );
	iFile.GetKeyDouble ( "Thread", "ThrMotionCycleTimeS", &m_Param.dCycleTime );
	LOGALERT("compiled for platform control");
#else
	iFile.SetFileName ( "Arm/IniFiles/ArmCtrlMotion.ini", "CanDriveHarmonica.cpp" );
	iFile.GetKeyDouble ( "Timing", "CycleTime", &m_Param.dCycleTime );
	LOGALERT("compiled for arm control");
#endif
*/

	m_iDivStatus = 0;
	m_iDigIn = 0;
}

void CanDriveHarmonica::setCycleTime(double time)
{
	m_Param.dCycleTime = time;
}


//-----------------------------------------------
void CanDriveHarmonica::setCANId ( int iID )
{
	m_ParamCANopen.iTxPDO1 = iID + 0x180;
	m_ParamCANopen.iRxPDO1 = iID + 0x200;

	m_ParamCANopen.iTxPDO2 = iID + 0x280;
	m_ParamCANopen.iRxPDO2 = iID + 0x300;

	m_ParamCANopen.iTxPDO3 = iID + 0x380;
	m_ParamCANopen.iRxPDO3 = iID + 0x400;

	m_ParamCANopen.iTxPDO4 = iID + 0x480;
	m_ParamCANopen.iRxPDO4 = iID + 0x500;

	m_ParamCANopen.iTxSDO = iID + 0x580;
	m_ParamCANopen.iRxSDO = iID + 0x600;

/* TODO:
#ifdef LOG_ENABLED
	char c[100];
	sprintf ( c, "Arm/Log/LogCAN%d", iID );
	//m_LogCAN.open ( c, LogFile::CONST_FILENAME );
#endif
#ifdef LOG_MSG_ENABLED
	char c[100];
	sprintf ( c, "Arm/Log/LogCANMsg%d", iID );
	//m_LogCANMsg.open ( c, LogFile::CONST_FILENAME );
#endif
*/

}

//-----------------------------------------------
bool CanDriveHarmonica::evalReceivedMsg ( CanMsg& msg )
{
	bool bRet;
	bool bPrint = false;
	int iPosCnt;
	int iStatusRegister, iMotorFailure;
	int iTemp1, iTemp2;

	m_CanMsgLast = msg;
	bRet = false;
	

	// eval answers from TPDO1 - transmitted on SYNC msg
	if ( msg.m_iID == m_ParamCANopen.iTxPDO1 )
	{
		iTemp1 = ( msg.getAt ( 3 ) << 24 ) | ( msg.getAt ( 2 ) << 16 )
		         | ( msg.getAt ( 1 ) << 8 ) | ( msg.getAt ( 0 ) );

		m_dPosWheelMeasRad = m_DriveParam.getSign() * m_DriveParam.convIncrToRad ( iTemp1 );

//#ifdef MEASURE_VELOCITY
/*		iTemp2 = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
		         | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );


		double velComp = ( m_dPosWheelMeasRad - m_dLastPos ) / m_Param.dCycleTime;
		m_dLastPos = m_dPosWheelMeasRad;
		m_dVelWheelMeasRadS = m_DriveParam.getSign() * m_DriveParam.convIncrPerPeriodToRadS ( iTemp2 );
		OUTPUTDEBUG("vel: %f %f itemp2: %i  %u, canid: %i type: %i, msg %i %i %i %i",m_dVelWheelMeasRadS, velComp ,iTemp2, iTemp2,
			m_DriveParam.getCANId(), msg.m_iType, (unsigned int)  msg.getAt ( 7 ), (unsigned int) msg.getAt ( 6 ), 
			(unsigned int) msg.getAt ( 5 ) , (unsigned int) msg.getAt ( 4 ));
		m_iCurrentMeasPromille = 0;
*/
// #else

		m_iCurrentMeasPromille = ( ( char ) msg.getAt ( 5 ) << 8 ) | msg.getAt ( 4 );
		m_iCurrentMeasPromille = abs ( m_iCurrentMeasPromille );


		m_dVelWheelMeasRadS = ( m_dPosWheelMeasRad - m_dLastPos ) / m_Param.dCycleTime;
		m_dLastPos = m_dPosWheelMeasRad;


		if( iTemp1 - last_pose_incr > 900000000 || iTemp1 - last_pose_incr < -900000000)  //measurment error due to pos counter overflow
		{
			OUTPUTERROR("cob_canopen_motor: position counter overflow, size: %i", iTemp1);
			m_dVelWheelMeasRadS = m_dLastVel;

		}
		m_dLastVel = m_dVelWheelMeasRadS;
		last_pose_incr = iTemp1;


		OUTPUTDEBUG("vel: %f itemp1: %i, canid: %i type: %i",m_dVelWheelMeasRadS, iTemp1,
			m_DriveParam.getCANId(), msg.m_iType);
//#endif


		m_dWatchdogTime = 0;
		bRet = true;
	}
	// eval answers from TPDO3 - transmitted on SYNC msg
	else if ( msg.m_iID == m_ParamCANopen.iTxPDO3 )
	{
		bRet = true;
	}
	// eval answers from binary interpreter
	if ( msg.m_iID == m_ParamCANopen.iTxPDO2 )
	{

		if ( m_DriveParam.getDriveIdent() == 0 )
		{
			int zz = 0;
		}

		if ( ( msg.getAt ( 0 ) == 'P' ) && ( msg.getAt ( 1 ) == 'X' ) ) // current pos
		{
			iPosCnt = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			          | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );

			m_dPosWheelMeasRad = m_DriveParam.getSign() * m_DriveParam.convIncrToRad ( iPosCnt );
			m_dVelWheelMeasRadS = ( m_dPosWheelMeasRad - m_dLastPos ) / m_Param.dCycleTime;
			m_dLastPos = m_dPosWheelMeasRad;

		}
		if ( ( msg.getAt ( 0 ) == 'V' ) && ( msg.getAt ( 1 ) == 'X' ) ) // current pos
		{
			iPosCnt = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			          | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );

			m_dVelWheelMeasRadS = ( (double) iPosCnt ) / m_DriveParam.getEncoderIncr() * m_DriveParam.getSign();

		}
		else if ( ( msg.getAt ( 0 ) == 'M' ) && ( msg.getAt ( 1 ) == 'O' ) )
		{
			iTemp1 = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			         | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );
		}

		else if ( ( msg.getAt ( 0 ) == 'I' ) && ( msg.getAt ( 1 ) == 'P' ) ) // digital in == limit switches
		{
			m_iDigIn = 0x1FFFFF & ( ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			                        | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) ) );
		}

		else if ( ( msg.getAt ( 0 ) == 'S' ) && ( msg.getAt ( 1 ) == 'R' ) )
		{
			iStatusRegister = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			                  | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );

			evalStatusRegister ( iStatusRegister );
		}

		else if ( ( msg.getAt ( 0 ) == 'M' ) && ( msg.getAt ( 1 ) == 'F' ) )
		{
			iMotorFailure = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			                | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );

			printMotorFailure ( iMotorFailure );
			OUTPUTDEBUG("motor failure %i ",iMotorFailure);
		}
		else
		{
			OUTPUTDEBUG("unhandeled can-msg of type: %c %c", msg.getAt ( 0 ), msg.getAt ( 1 ));
		}

		m_dWatchdogTime = 0;
		bRet = true;
	}
	// eval answer from SDO
	else if ( msg.m_iID == m_ParamCANopen.iTxSDO )
	{
/*
TODO: This function overwrites m_dLastPos with wrong value after homing procedure
		iPosCnt = m_DriveParam.getSign() * getSDODataInt32 ( msg );
		m_dPosWheelMeasRad = m_DriveParam.convIncrToRad ( iPosCnt );
		m_dVelWheelMeasRadS = ( m_dPosWheelMeasRad - m_dLastPos ) / m_Param.dCycleTime;
		m_dLastPos = m_dPosWheelMeasRad;
*/

		LOGINFO("CanDriveHarmonica: " << m_ParamCANopen.iTxPDO2 << ": pos = " << m_dPosWheelMeasRad);

		m_dWatchdogTime = 0;
		bRet = true;
		//bPrint = true;
	}
	else
	{
		bPrint = true;
	}

/* TODO:
#ifdef LOG_MSG_ENABLED
	if ( bPrint )
	{
		int iIndex = ( msg.getAt ( 2 ) << 8 ) | msg.getAt ( 1 );
		int iSubIndex = msg.getAt ( 3 );
		//sprintf ( //m_LogCANMsg.m_cBuf, "t:%d id:%d l:%d i:%d si:%d d0:%d d1:%d d2:%d d3:%d\n",
		          msg.m_iType, msg.m_iID, msg.m_iLen,
		          iIndex, iSubIndex,
		          msg.getAt ( 4 ), msg.getAt ( 5 ), msg.getAt ( 6 ), msg.getAt ( 7 ) );

		//m_LogCANMsg.print ( true, false );
	}
#endif
*/

	return bRet;
}


bool CanDriveHarmonica::setTypeMotion(int iType)
{

	if (iType == 0)
	{
		// 1.) Switch to UnitMode = 5 (Single Loop Position Control) //
	
		// switch off Motor to change Unit-Mode
		IntprtSetInt(8, 'M', 'O', 0, 0);
		usleep(20000);
		// switch Unit-Mode
		IntprtSetInt(8, 'U', 'M', 0, 5);
			
		// set Target Radius to X Increments
		IntprtSetInt(8, 'T', 'R', 1, 15);
		// set Target Time to X ms
		IntprtSetInt(8, 'T', 'R', 2, 100);
		usleep(100000);
		
		
	}
	else if (iType == 1)
	{
		// Switch to TorqueControl-Mode
		// switch off Motor to change Unit-Mode
		IntprtSetInt(8, 'M', 'O', 0, 0);
		usleep(50000);
		// switch Unit-Mode 1: Torque Controlled
		IntprtSetInt(8, 'U', 'M', 0, 1);
		// disable external compensation input
		// to avoid noise from that input pin
		IntprtSetInt(8, 'R', 'M', 0, 0);

		// debugging:
		//std::cout << "Motor"<<m_DriveParam.getDriveIdent()<<" Unit Mode switched to: TORQUE controlled" << std::endl;
		usleep(100000);
	}
	else
	{
		//Default Motion Type = VelocityControled
		// switch off Motor to change Unit-Mode
		IntprtSetInt(8, 'M', 'O', 0, 0);
		// switch Unit-Mode
		IntprtSetInt(8, 'U', 'M', 0, 2);
		// set profiler Mode (only if Unit Mode = 2)
		IntprtSetInt(8, 'P', 'M', 0, 1);
		usleep(100000);
	}
	
	return true;
}

//-----------------------------------------------
bool CanDriveHarmonica::init ( bool bZeroPosCnt )
{
	m_iMotorState = ST_PRE_INITIALIZED;

	//Disabling motor
	IntprtSetInt ( 8, 'M', 'O', 0, 0, true );

	if ( bZeroPosCnt )
	{
		IntprtSetInt ( 8, 'P', 'X', 0, 0, true );
	}

	// set acceleration and deceleration
	IntprtSetInt ( 8, 'A', 'C', 0, ( int ) m_DriveParam.getAcc(), true );
	IntprtSetInt ( 8, 'D', 'C', 0, ( int ) m_DriveParam.getDec(), true );

	// stop deceleration for emstop
	IntprtSetInt ( 8, 'S', 'D', 0, ( int ) ( m_DriveParam.getDec() * 5 ), true );

	// speed for PTP motion
	IntprtSetInt ( 8, 'S', 'P', 0, ( int ) m_DriveParam.getVelPosMode(), true );

	// set mapping of TPDO1: actual position, actual velocity
	// map actual position to 4 bytes of TPDO1
	// parameter 0x60640020
	// 0x6064 = index
	// 0x00 = sub index
	// 0x20 = number of bits
	sendSDODownload ( 0x1A00, 0, 0, true );	// stop all emissions of TPDO1
	sendSDODownload ( 0x1A00, 1, 0x60640020, true );	// map actual position to 4 bytes of TPDO1

// #ifdef MEASURE_VELOCITY
//	sendSDODownload ( 0x1A00, 2, 0x60690020, true );	// map actual velocity to 4 bytes of TPDO1
//#else
	sendSDODownload ( 0x1A00, 2, 0x60780010, true );	// map actual current (in promille of rated current) to 2 bytes of TPDO1
//#endif

	sendSDODownload ( 0x1800, 2, 1, true );	// transmission type "synch"
	sendSDODownload ( 0x1A00, 0, 2, true );	// activate mapped objects

	return true;
}

//-----------------------------------------------
bool CanDriveHarmonica::start()
{
	const int ciMaxMsg = 10;

	bool bRet;
	CanMsg msg;

	m_iDigIn = 0;

	while ( m_pCanCtrl->receiveMsg ( &msg ) );

	IntprtSetInt ( 8, 'M', 'O', 0, 1, true );
	
	m_iMotorState = ST_OPERATION_ENABLED;
	bRet = true;

/*	
	IntprtSetInt ( 4, 'S', 'R', 0, 0, true );

	Sleep(300);

	int i = 0;
	int iStatus = -1;
	while ( ( m_pCanCtrl->receiveMsg ( &msg ) ) && ( i < 300 ) )
	{
		if ( ( msg.getAt ( 0 ) == 'S' ) && ( msg.getAt ( 1 ) == 'R' ) )
		{
			iStatus = ( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 )
			          | ( msg.getAt ( 5 ) << 8 ) | ( msg.getAt ( 4 ) );

			bRet = evalStatusRegister ( iStatus );

			break;
		}

		i++;

		Sleep (10);
	}

	if ( (iStatus == -1) || (i == 300) )
	{
		LOGERROR ( "start(): no answer on status request" );
		bRet = false;
	}
	
*/
	// start watchdog timer
	m_dWatchdogTime = 0;

	return bRet;
}

//-----------------------------------------------
void CanDriveHarmonica::stop()
{
	IntprtSetInt ( 8, 'M', 'O', 0, 0, false );
}

//-----------------------------------------------
bool CanDriveHarmonica::shutdown()
{
	LOGINFO ( "shutdown(), drive: " << m_DriveParam.getDriveIdent() );

	IntprtSetInt ( 8, 'M', 'O', 0, 0, true );

	return true;
}

//-----------------------------------------------
double CanDriveHarmonica::getTimeToLastMsg()
{
	m_dWatchdogTime +=  m_Param.dCycleTime;
	return m_dWatchdogTime;
}

//-----------------------------------------------
bool CanDriveHarmonica::prepareHoming()
{
	bool bHomingSwitchActive;
	double dHomeVel;
	int iDigIn, iHomeDigIn;
	CanMsg msg;

	if ( !m_DriveParam.getHoming() || ( m_DriveParam.getHomeEvent() >= 1000 ) )
	{
		return false;
	}

	dHomeVel = m_DriveParam.getHomeVel();
	iHomeDigIn = m_DriveParam.getHomeDigIn();

	// clear receive buffer
	while ( m_pCanCtrl->receiveMsg ( &msg ) );

	bHomingSwitchActive = true;
	do
	{
		IntprtSetInt ( 4, 'I', 'P', 0, 16, true );

		m_pCanCtrl->receiveMsgRetry ( &msg, 10 );

		if ( ( msg.getAt ( 0 ) == 'I' ) && ( msg.getAt ( 1 ) == 'P' ) )
		{
			iDigIn = 0x1FFFFF & (	( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 ) |
			                      ( msg.getAt ( 5 ) << 8 ) | msg.getAt ( 4 ) );

			if ( ( iDigIn & iHomeDigIn ) == 0x0000 )
			{
				bHomingSwitchActive = false;
			}
			else
			{
				int sign = m_DriveParam.getSign();
				// try to move out of homing switch
				IntprtSetInt ( 8, 'J', 'V', 0, int ( - dHomeVel ) * sign, true );
				IntprtSetInt ( 4, 'B', 'G', 0, 0, true );
			}
		}
	}
	while ( bHomingSwitchActive );
	// stop motor
	usleep(100000); //wait some time to make sure that limit switch is inactive
	IntprtSetInt ( 8, 'J', 'V', 0, 0, true );
	IntprtSetInt ( 4, 'B', 'G', 0, 0, true );
	return true;
}

//-----------------------------------------------
bool CanDriveHarmonica::initHoming(bool keepDriving = false)
{
	int iHomeEvent;
	double dHomePos;

	if ( !m_DriveParam.getHoming() || ( m_DriveParam.getHomeEvent() >= 1000 ) )
	{
		return true;
	}

	dHomePos = m_DriveParam.getHomePos();
	iHomeEvent = m_DriveParam.getHomeEvent();

	// value to load at homing event
	IntprtSetInt ( 8, 'H', 'M', 2, ( int ) dHomePos, true );

	// see HomeEvent in header
	IntprtSetInt ( 8, 'H', 'M', 3, iHomeEvent, true );

	if(keepDriving)
	{
		// HM[4] = 2 : keep driving
		IntprtSetInt(8, 'H', 'M', 4, 2);
	}
	else
	{
		// after event stop immediately
		IntprtSetInt ( 8, 'H', 'M', 4, 0, true );
	}
	// arm homing process
	IntprtSetInt ( 8, 'H', 'M', 1, 1, true );
	// absolute setting of position counter: PX = HM[2]
	IntprtSetInt ( 8, 'H', 'M', 5, 0, true );



	return true;
}



int getNearest(double num, double cmp[], int iSize, double* pdTol)
{
	int i, iMinIndex;
	double dDist, dMinDist;

	dMinDist = 1E100;
	for(i = 0; i < iSize; i++)
	{
		dDist = fabs(num - cmp[i]);
		if(dDist < dMinDist)
		{
			dMinDist = dDist;
			iMinIndex = i;
		}
	}

	*pdTol = dMinDist;

	return iMinIndex;
}

//-----------------------------------------------
bool CanDriveHarmonica::execHoming()
{
	const double cdHomeTimeOut = 10;

	bool bNoHomeTimeOut;
	int i, iIndex;
	double dHomeVel;
	CanMsg msg;
	TimeStamp tsStart, tsNow;
	TimeStamp tsLowHigh, tsHighLow;
	char c[100];
	double dBlockWidthMeas;
	double dAngMeas, dTol;
	double dBlockWidth[20];
	double dBlockAngRad[20];

	if ( !m_DriveParam.getHoming() )
	{
		return true;
	}
/*TODO: parameterers for HomeEvent: (1000 && 1001)
	if ( m_DriveParam.getHomeEvent() == 1000 ) // homing with Neobotix variable block width ring
	{
		LOGINFO ( "block width ring homing started" );

		IniFile iFile;
		iFile.SetFileName ( "Arm/IniFiles/AbsBlockEnc.ini", "CanDriveHarmonica.cpp" );
		for ( i = 0; i < 20; i++ )
		{
			sprintf ( c, "B%d", i );
			iFile.GetKeyDouble ( "BlockWidth", c, &dBlockWidth[i] );
			sprintf ( c, "A%d", i );
			iFile.GetKeyDouble ( "Angle", c, &dBlockAngRad[i] );
		}

		// set homing velocity
		dHomeVel = m_DriveParam.getHomeVel();
		IntprtSetInt ( 8, 'J', 'V', 0, -10000, true );
		IntprtSetInt ( 4, 'B', 'G', 0, 0, true );

		// clear CAN buffer
		while ( m_pCanCtrl->receiveMsg ( &msg ) );

		// wait for input low
		bNoHomeTimeOut = waitOnDigIn ( 0, 1, m_DriveParam.getHomeTimeOut() );
		if ( !bNoHomeTimeOut ) { return false; }
		LOGINFO("input low, init found");

		// find transition low - high
		bNoHomeTimeOut = waitOnDigIn ( 1, 1, m_DriveParam.getHomeTimeOut() );
		if ( !bNoHomeTimeOut ) { return false; }
		LOGINFO("input high, start found");
		tsLowHigh.SetNow();

		// find transition high - low
		bNoHomeTimeOut = waitOnDigIn ( 0, 1, m_DriveParam.getHomeTimeOut() );
		if ( !bNoHomeTimeOut ) { return false; }
		LOGINFO("input low, end found");
		tsHighLow.SetNow();

		dBlockWidthMeas = tsHighLow - tsLowHigh;
		iIndex = getNearest ( dBlockWidthMeas, dBlockWidth, 20, &dTol );
		dAngMeas = dBlockAngRad[iIndex];

		int iIncr = ( int ) m_DriveParam.convRadToIncr ( dAngMeas );

		LOGINFO ( "found angle:" << dAngMeas << " incr:" << iIncr << " with tolerance " << dTol );

		// write value immediately
		IntprtSetInt ( 8, 'H', 'M', 2, iIncr, true );	// value to load at homing event
		IntprtSetInt ( 8, 'H', 'M', 3, 0, true );		// trigger immediately on arm homing process
		IntprtSetInt ( 8, 'H', 'M', 4, 0, true );		// after event stop immediately
		IntprtSetInt ( 8, 'H', 'M', 5, 0, true );		// absolute setting of position counter: PX = HM[2]
		IntprtSetInt ( 8, 'H', 'M', 1, 1, true );		// arm homing process
	}
	else if ( m_DriveParam.getHomeEvent() == 1001 ) // block width ring calibration
	{
		execBlockWidthMeas();
	}
	else
	{
*/
	dHomeVel = m_DriveParam.getHomeVel();
	int sign = m_DriveParam.getSign();

	// set homing velocity
	IntprtSetInt ( 8, 'J', 'V', 0, int ( dHomeVel * sign), true );
	IntprtSetInt ( 4, 'B', 'G', 0, 0, true );

	LOGINFO ( "drive " << m_DriveParam.getDriveIdent() << " homing started" );
//	}

	return true;
}

//-----------------------------------------------
bool CanDriveHarmonica::isHomingFinished(bool waitTillHomed)
{
	bool bHomeTimeOut;
	bool bLimSwRight;
	int iDigIn;
	int iHomeDigIn; // 0x0400 for arm scara, 0x0001 for three wheel kin
	CanMsg msg;
	TimeStamp tsStart, tsNow;

	if ( !m_DriveParam.getHoming() || ( m_DriveParam.getHomeEvent() >= 1000 ) )
	{
		return true;
	}

	iHomeDigIn = m_DriveParam.getHomeDigIn();

	// clear CAN buffer
	while ( m_pCanCtrl->receiveMsg ( &msg ) );

	tsStart.SetNow();
	bLimSwRight = false;
	do
	{
		IntprtSetInt ( 4, 'I', 'P', 0, 16, true );

		m_pCanCtrl->receiveMsgRetry ( &msg, 10 );

		if ( ( msg.getAt ( 0 ) == 'I' ) && ( msg.getAt ( 1 ) == 'P' ) )
		{
			iDigIn = 0x1FFFFF & (	( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 ) |
			                      ( msg.getAt ( 5 ) << 8 ) | msg.getAt ( 4 ) );

			if ( ( iDigIn & iHomeDigIn ) != 0x0000 )
			{
				bLimSwRight = true;
			}
		}

		tsNow.SetNow();
		bHomeTimeOut = true;
		if ( ( tsNow - tsStart < m_DriveParam.getHomeTimeOut() ) ||
		        ( m_DriveParam.getHomeTimeOut() == 0 ) )
		{
			bHomeTimeOut = false;
		}
	}
	while ( !bLimSwRight && !bHomeTimeOut &&  waitTillHomed);

	if ( bLimSwRight )
	{
		LOGINFO ( "homing " << m_DriveParam.getDriveIdent() << " ok" );

		return true;
	}
	else if(bHomeTimeOut)
	{
		LOGERROR ( "homing " << m_DriveParam.getDriveIdent() << " timeout" );
		IntprtSetInt ( 8, 'J', 'V', 0, 0, true );
		IntprtSetInt ( 4, 'B', 'G', 0, 0, true );

		return false;
	}
	else
	{
		return false;
	}

}

//-----------------------------------------------
void CanDriveHarmonica::exitHoming ( double t, bool keepDriving)
{
	double dHomeVel;

	if ( !m_DriveParam.getHoming() || ( m_DriveParam.getHomeEvent() >= 1000 ) )
	{
		return;
	}

	dHomeVel = m_DriveParam.getHomeVel();

	// set homing velocity
	IntprtSetInt ( 8, 'J', 'V', 0, int ( dHomeVel ), true );
	IntprtSetInt ( 4, 'B', 'G', 0, 0, true );

	Sleep ( ( double ) t );

	if(!keepDriving)
	{
		IntprtSetInt ( 8, 'J', 'V', 0, 0, true );
		IntprtSetInt ( 4, 'B', 'G', 0, 0, true );
	}
}

//-----------------------------------------------
void CanDriveHarmonica::initWatchdog()
{
	LOGINFO ( "drive " << m_DriveParam.getDriveIdent() << " init watchdog" );
	m_dWatchdogTime = 0; //first reset watchdog on this program layer then start watchdogs on motor drivers

	const int c_iHeartbeatTimeMS = 2000;
	const int c_iNMTNodeID = 0x00;

	// consumer (PC) heartbeat time
	sendSDODownload ( 0x1016, 1, ( c_iNMTNodeID << 16 ) | c_iHeartbeatTimeMS, true );

	// error behavior after failure: 0=pre-operational, 1=no state change, 2=stopped"
	// ovb: changed so that motors stay in the loop sendSDODownload(0x1029, 1, 2, true);
	sendSDODownload ( 0x1029, 1, 1, true );

	// motor behavior after heartbeat failure: "quick stop"
	sendSDODownload ( 0x6007, 0, 3, true );

	// activate emergency events: "heartbeat event"
	sendSDODownload ( 0x2F21, 0, 0x08, true );
}

//-----------------------------------------------
void CanDriveHarmonica::setModuloCount ( double min, double max )
{
	int iPosMinIncr, iPosMaxIncr;

	iPosMinIncr = ( int ) m_DriveParam.convRadToIncr ( min );
	iPosMaxIncr = ( int ) m_DriveParam.convRadToIncr ( max );

	if ( ( iPosMaxIncr - iPosMinIncr ) % 2 != 0 )
	{
		iPosMaxIncr++;
	}

	IntprtSetInt ( 8, 'M', 'O', 0, 0, true );

	// set the maximum tracking error to be smaller than (max - min)
	IntprtSetInt ( 8, 'E', 'R', 3, 6000, true );

	// set modulo count limits
	IntprtSetInt ( 8, 'X', 'M', 2, iPosMaxIncr, true );
	IntprtSetInt ( 8, 'X', 'M', 1, iPosMinIncr, true );

	IntprtSetInt ( 8, 'M', 'O', 0, 1, true );
}

//-----------------------------------------------
bool CanDriveHarmonica::setWheelPosVel ( double dPosWheel, double dVelWheel, bool bBeginMotion, bool bUsePosMode )
{
	int iRet;
	double dPosEncIncr, dVelEncIncrPeriod;
	CanMsg msg;

	dPosEncIncr = m_DriveParam.convRadToIncr ( dPosWheel );
	dVelEncIncrPeriod = m_DriveParam.convRadSToIncrPerPeriod ( dVelWheel );

	dPosEncIncr *= m_DriveParam.getSign();
	dVelEncIncrPeriod *= m_DriveParam.getSign();

	double dUnlimitedEncIncPeriod = dVelEncIncrPeriod;
	iRet = MathSup::limit ( &dVelEncIncrPeriod, m_DriveParam.getVelMax() );
	if ( iRet != 0 )
	{
		LOGINFO("v:" << m_DriveParam.getDriveIdent() << " " << dUnlimitedEncIncPeriod << " set to 0. Max = " << m_DriveParam.getVelMax() );
		dVelEncIncrPeriod = 0;
	}

	// if vel == 0, use pos ctrl, else use vel ctrl
	if ( ( dVelEncIncrPeriod == 0 ) && bUsePosMode )
	{
		IntprtSetInt ( 8, 'P', 'A', 0, ( int ) dPosEncIncr );
	}
	else
	{
		IntprtSetInt ( 8, 'J', 'V', 0, ( int ) dVelEncIncrPeriod );
	}

	// for multi drive application 'B' 'G' should be called for all drives by the group ID
	if ( bBeginMotion )
	{
		IntprtSetInt ( 4, 'B', 'G', 0, 0 );
	}

	// request status
	m_iDivStatus++;
	if ( m_iDivStatus == m_Param.iDivForRequestStatus )
	{
		IntprtSetInt ( 4, 'I', 'P', 0, 16, false );
	}
	else if ( m_iDivStatus >= 2 * m_Param.iDivForRequestStatus )
	{
		requestStatus();
		m_iDivStatus = 0;
	}

//log
/* TODO:
#ifdef LOG_ENABLED
	//sprintf ( m_LogCAN.m_cBuf, "%d %f %f %d\n", counter, dPosWheel, dVelWheel, bUsePosMode);
	//m_LogCAN.print ( true, false );
#endif
*/

	return ( iRet == 0 );
}

//-----------------------------------------------
bool CanDriveHarmonica::setWheelVel ( double dVelWheel, bool bQuickStop, bool bBeginMotion )
{
	int iRet;
	double dVelEncIncrPeriod;

	dVelEncIncrPeriod = m_DriveParam.getSign() * m_DriveParam.convRadSToIncrPerPeriod ( dVelWheel );
	LOGINFO("v:" << m_DriveParam.getDriveIdent() << " " << dVelEncIncrPeriod);
	iRet = MathSup::limit ( &dVelEncIncrPeriod, m_DriveParam.getVelMax() );
	if ( iRet != 0 )
	{
		//LOGINFO("v:" << m_DriveParam.getDriveIdent() << " " << iVelEncIncrPeriod << " lim. to " << m_DriveParam.getVelMax() );
		dVelEncIncrPeriod = 0;
	}

	IntprtSetInt ( 8, 'J', 'V', 0, ( int ) dVelEncIncrPeriod );

	// for multi drive application 'B' 'G' should be called for all drives by the group ID
	if ( bBeginMotion )
	{
		IntprtSetInt ( 4, 'B', 'G', 0, 0 );
	}

	// request status
	if ( ++m_iDivStatus > m_Param.iDivForRequestStatus )
	{
		requestStatus();
		m_iDivStatus = 0;
	}

	return ( iRet == 0 );
}

void CanDriveHarmonica::requestPosVel()
{
	//IntprtSetInt ( 8, 'V', 'X', 0, 0, true );
}

//-----------------------------------------------
void CanDriveHarmonica::getWheelPos ( double* dWheelPos )	//also requests status
{
	*dWheelPos = m_dPosWheelMeasRad;
}

//-----------------------------------------------
void CanDriveHarmonica::getWheelPosVel ( double* pdAngWheel, double* pdVelWheel )
{
	*pdAngWheel = m_dPosWheelMeasRad;
	*pdVelWheel = m_dVelWheelMeasRadS;
}

//-----------------------------------------------
void CanDriveHarmonica::getWheelDltPosVel ( double* pdDltAngWheel, double* pdVelWheel )
{
	*pdDltAngWheel = m_dPosWheelMeasRad - m_dAngleWheelRadMem;
	*pdVelWheel = m_dVelWheelMeasRadS;
	m_dAngleWheelRadMem = m_dPosWheelMeasRad;
}

//-----------------------------------------------
void CanDriveHarmonica::getData ( double* pdPosWheelRad, double* pdVelWheelRadS,
                                  int* piCurrentPromille, int* piStatus )
{
	*piStatus &= 0xFFE00000;
	*piStatus |= m_iStatusCtrl;

	*pdPosWheelRad = m_dPosWheelMeasRad;
	*pdVelWheelRadS = m_dVelWheelMeasRadS;
	*piCurrentPromille = m_iCurrentMeasPromille;
}

//-----------------------------------------------
void CanDriveHarmonica::getStatus (	int* piStatus,
                                    int* piCurrentMeasPromille,
                                    int* piTempCel )
{
	*piStatus &= 0xFFE00000;
	*piStatus |= m_iStatusCtrl;

	*piCurrentMeasPromille = m_iCurrentMeasPromille;
	*piTempCel = 0;
}

//-----------------------------------------------
void CanDriveHarmonica::requestStatus()
{
	IntprtSetInt ( 4, 'S', 'R', 0, 0 );
}

//-----------------------------------------------
bool CanDriveHarmonica::isError(int* piDigIn)
{
	double dWatchTime;

	*piDigIn = m_iDigIn;

	dWatchTime = getTimeToLastMsg();
	if ( ( dWatchTime >= m_Param.dCANTimeout ) && ( m_iCommState == 0 ) )
	{
		LOGERROR ( "motor " << m_DriveParam.getDriveIdent() << " no CAN communication for " << dWatchTime << " s" );
		m_iCommState = -1;
	}

	if ( ( dWatchTime < m_Param.dCANTimeout ) && ( m_iCommState == -1 ) )
	{
		LOGINFO ( "motor " << m_DriveParam.getDriveIdent() << " CAN communication ok" );
		m_iCommState = 0;
	}

	/*if (m_iMotorState != ST_MOTOR_FAILURE)
	{
		// Check timeout of CAN communication
		double dWatchTime = getTimeToLastMsg();

		if (dWatchTime > m_Param.dCANTimeout)
		{
			LOGERROR("isError(), motor " << m_DriveParam.getDriveIdent() <<
				" no CAN communiction for " << dWatchTime << " s." );
			m_iMotorState = ST_MOTOR_FAILURE;
			m_FailureStartTime.SetNow();
		}
	}*/

	if ( m_iMotorState == ST_OPERATION_ENABLED  &&  !critical_state )
	{
		return false;
	}
	else
	{
		return true;
	}

}

//-----------------------------------------------
void CanDriveHarmonica::IntprtSetInt (	int iDataLen,
                                       char cCmdChar1, char cCmdChar2,
                                       int iIndex,
                                       int iData,
                                       bool bDelay )
{
	char cIndex[2];
	char cInt[4];
	CanMsg CMsgTr;

	CMsgTr.m_iID = m_ParamCANopen.iRxPDO2;
	CMsgTr.m_iLen = iDataLen;

	cIndex[0] = iIndex;
	cIndex[1] = iIndex >> 8;

	cInt[0] = iData;
	cInt[1] = iData >> 8;
	cInt[2] = iData >> 16;
	cInt[3] = iData >> 24;

	CMsgTr.set ( cCmdChar1, cCmdChar2, cIndex[0], cIndex[1], cInt[0], cInt[1], cInt[2], cInt[3] );
	m_pCanCtrl->transmitMsg ( CMsgTr );

	if ( bDelay )
	{
		Sleep ( 20 );
	}
}

//-----------------------------------------------
void CanDriveHarmonica::IntprtSetFloat ( int iDataLen,
        char cCmdChar1, char cCmdChar2,
        int iIndex,
        float fData,
        bool bDelay )
{
	char cIndex[2];
	char cFloat[4];
	CanMsg CMsgTr;
	char* pTempFloat = NULL;

	CMsgTr.m_iID = m_ParamCANopen.iRxPDO2;
	CMsgTr.m_iLen = iDataLen;

	cIndex[0] = iIndex;
	cIndex[1] = ( iIndex >> 8 ) & 0x3F;  // MMB/23.02.2006: The two MSB must be 0. Cf. DSP 301 Implementation guide p. 39.

	pTempFloat = ( char* ) &fData;
	for ( int i=0; i<4; i++ )
		cFloat[i] = pTempFloat[i];

	CMsgTr.set ( cCmdChar1, cCmdChar2, cIndex[0], cIndex[1], cFloat[0], cFloat[1], cFloat[2], cFloat[3] );
	m_pCanCtrl->transmitMsg ( CMsgTr );

	if ( bDelay )
	{
		Sleep ( 20 );
	}
}

//-----------------------------------------------
void CanDriveHarmonica::sendSDOUpload ( int iObjIndex, int iObjSubIndex )
{
	CanMsg CMsgTr;
	const int ciInitUploadReq = 0x40;

	CMsgTr.m_iLen = 8;
	CMsgTr.m_iID = m_ParamCANopen.iRxSDO;

	unsigned char cMsg[8];

	cMsg[0] = ciInitUploadReq;
	cMsg[1] = iObjIndex;
	cMsg[2] = iObjIndex >> 8;
	cMsg[3] = iObjSubIndex;
	cMsg[4] = 0x00;
	cMsg[5] = 0x00;
	cMsg[6] = 0x00;
	cMsg[7] = 0x00;

	CMsgTr.set ( cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7] );
	m_pCanCtrl->transmitMsg ( CMsgTr );
}

//-----------------------------------------------
void CanDriveHarmonica::sendSDODownload ( int iObjIndex, int iObjSubIndex, int iData, bool bDelay )
{
	CanMsg CMsgTr;

	const int ciInitDownloadReq = 0x20;
	const int ciNrBytesNoData = 0x00;
	const int ciExpedited = 0x02;
	const int ciDataSizeInd = 0x01;

	CMsgTr.m_iLen = 8;
	CMsgTr.m_iID = m_ParamCANopen.iRxSDO;

	unsigned char cMsg[8];

	cMsg[0] = ciInitDownloadReq | ( ciNrBytesNoData << 2 ) | ciExpedited | ciDataSizeInd;
	cMsg[1] = iObjIndex;
	cMsg[2] = iObjIndex >> 8;
	cMsg[3] = iObjSubIndex;
	cMsg[4] = iData;
	cMsg[5] = iData >> 8;
	cMsg[6] = iData >> 16;
	cMsg[7] = iData >> 24;

	CMsgTr.set ( cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7] );
	m_pCanCtrl->transmitMsg ( CMsgTr );

	if ( bDelay )
	{
		Sleep ( 20 );
	}
}

//-----------------------------------------------
void CanDriveHarmonica::evalSDO ( CanMsg& CMsg, int* pIndex, int* pSubindex )
{
	*pIndex = ( CMsg.getAt ( 2 ) << 8 ) | CMsg.getAt ( 1 );
	*pSubindex = CMsg.getAt ( 3 );
}

//-----------------------------------------------
int CanDriveHarmonica::getSDODataInt32 ( CanMsg& CMsg )
{
	int iData = ( CMsg.getAt ( 7 ) << 24 ) | ( CMsg.getAt ( 6 ) << 16 ) |
	            ( CMsg.getAt ( 5 ) << 8 ) | CMsg.getAt ( 4 );

	return iData;
}

//-----------------------------------------------
bool CanDriveHarmonica::evalStatusRegister ( int iStatus )
{
	int iNumDrive;
	bool bNoError;

	bNoError = false;
	iNumDrive = m_DriveParam.getDriveIdent();
	// bit 0 or bit 6 set? if true set motor state to failure
	if ( MathSup::isBitSet ( iStatus, 0 ) || MathSup::isBitSet ( iStatus, 6 ) )
	{
		m_iNewMotorState = ST_MOTOR_FAILURE;
	}

	// set m_iStatusCtrl bits
	critical_state = false;
	if ( MathSup::isBitSet ( iStatus, 0 ) )
	{
		if ( ( ( iStatus & 0x0E ) == 2 ) && !MathSup::isBitSet ( m_iStatusCtrl, 18 ) )
		{
			OUTPUTERROR(" SR-under voltage %i", iNumDrive);
			m_iStatusCtrl |= 0x00020000;
			//critical_state = true;
		}

		if ( ( ( iStatus & 0x0E ) == 4 ) && !MathSup::isBitSet ( m_iStatusCtrl, 19 ) )
		{
			OUTPUTERROR( " SR-over voltage %i", iNumDrive );
			m_iStatusCtrl |= 0x00040000;
			critical_state = true;
		}

		if ( ( ( iStatus & 0x0E ) == 10 ) && !MathSup::isBitSet ( m_iStatusCtrl, 20 ) )
		{
			OUTPUTERROR( " SR-short circuit %i ", iNumDrive );
			m_iStatusCtrl |= 0x00080000;
			critical_state = true;
		}

		if ( ( ( iStatus & 0x0E ) == 12 ) && !MathSup::isBitSet ( m_iStatusCtrl, 21 ) )
		{
			OUTPUTERROR(" SR-overheating %i", iNumDrive );
			m_iStatusCtrl |= 0x00100000;
			critical_state = true;
		}
	}
	else if ( MathSup::isBitSet ( iStatus, 6 ) )
	{
		// general failure, request detailed description
		if ( m_iMotorState != ST_MOTOR_FAILURE )
		{
			IntprtSetInt ( 4, 'M', 'F', 0, 0 );
		}
	}
	else
	{
		m_iStatusCtrl = CanDriveItf::DRIVE_NO_ERROR;

		if ( MathSup::isBitSet ( iStatus, 4 ) )
		{
			if ( m_iMotorState != ST_OPERATION_ENABLED )
			{
				OUTPUTDEBUG ( " operation enabled %i", iNumDrive );
			}

			m_iNewMotorState = ST_OPERATION_ENABLED;

			bNoError = true;
		}
		else
		{
			if ( m_iMotorState != ST_OPERATION_DISABLED )
			{
				OUTPUTDEBUG ( " operation disabled %i", iNumDrive );
			}

			m_iNewMotorState = ST_OPERATION_DISABLED;
		}

		// Current limit
		if ( MathSup::isBitSet ( iStatus, 13 ) )
		{
			if ( m_bCurrentLimitOn == false )
			{
				OUTPUTERROR ( " current limit on %i", iNumDrive);
			}

			m_bCurrentLimitOn = true;
		}
		else
		{
			m_bCurrentLimitOn = false;
		}
	}

	// Change state
	m_iMotorState = m_iNewMotorState;

	return bNoError;
}

//-----------------------------------------------
void CanDriveHarmonica::printMotorFailure ( int iFailure )
{
	int iNumDrive;

	iNumDrive = m_DriveParam.getDriveIdent();

	if ( iFailure == 0 )
	{
		LOGINFO ( iNumDrive << " no error" );
	}

	if ( MathSup::isBitSet ( iFailure, 2 ) )
	{
		LOGERROR ( iNumDrive << " feedback loss" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_FEEDBACK_LOSS;
	}

	if ( MathSup::isBitSet ( iFailure, 3 ) )
	{
		LOGALERT ( iNumDrive << " peak current exceeded" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_PEAK_CURRENT_EXCEEDED;
	}

	if ( MathSup::isBitSet ( iFailure, 4 ) )
	{
		LOGERROR ( iNumDrive << " inhibit" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_INHIBIT;
	}

	if ( MathSup::isBitSet ( iFailure, 6 ) )
	{
		LOGERROR ( iNumDrive << " Hall sensor error" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_HALL_ERROR;
	}

	if ( MathSup::isBitSet ( iFailure, 7 ) )
	{
		LOGERROR ( iNumDrive << " speed track error" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_SPEED_TRACK_ERROR;
	}

	if ( MathSup::isBitSet ( iFailure, 8 ) )
	{
		LOGERROR ( iNumDrive << " position track error" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_POS_TRACK_ERROR;
	}

	if ( MathSup::isBitSet ( iFailure, 9 ) )
	{
		LOGERROR ( iNumDrive << " inconsistent database" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_INCONS_DATABASE;
	}
	else
	{
		m_iStatusCtrl &= ( ~CanDriveItf::DRIVE_INCONS_DATABASE );
	}

	if ( MathSup::isBitSet ( iFailure, 11 ) )
	{
		LOGERROR ( iNumDrive << " heartbeat failure" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_HEARTBEAT_FAIL;
	}

	if ( MathSup::isBitSet ( iFailure, 12 ) )
	{
		LOGERROR(iNumDrive << " servo drive fault");
		m_iStatusCtrl |= CanDriveItf::ELOMC_SERVO_DRIVE_FAULT;
	}

	if ( ( iFailure & 0x0E000 ) == 0x2000 )
	{
		LOGERROR ( iNumDrive << " under voltage" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_UNDER_VOLTAGE;
	}

	if ( ( iFailure & 0x0E000 ) == 0x4000 )
	{
		LOGERROR ( iNumDrive << " over voltage" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_OVER_VOLTAGE;
	}

	if ( ( iFailure & 0x0E000 ) == 0xA000 )
	{
		LOGERROR ( iNumDrive << " short circuit" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_SHORT_CIRCUIT;
	}

	if ( ( iFailure & 0x0E000 ) == 0xC000 )
	{
		LOGERROR ( iNumDrive << " over temp" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_OVER_TEMP;
	}

	if ( MathSup::isBitSet ( iFailure, 16 ) )
	{
		LOGERROR ( iNumDrive << " electrical zero not found" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_ELECTRICAL_ZERO_NOT_FOUND;
	}

	if ( MathSup::isBitSet ( iFailure, 17 ) )
	{
		LOGERROR ( iNumDrive << " speed limit exceeded" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_SPEED_LIMIT_EXCEEDED;
	}

	if ( MathSup::isBitSet ( iFailure, 21 ) )
	{
		LOGERROR ( iNumDrive << " motor stuck" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_MOTOR_STUCK;
	}

	if ( MathSup::isBitSet ( iFailure, 22 ) )
	{
		LOGERROR ( iNumDrive << " position limit excceded" );
		m_iStatusCtrl |= CanDriveItf::DRIVE_POS_LIMIT_EXCEEDED;
	}

}

//-----------------------------------------------
bool CanDriveHarmonica::execBlockWidthMeas()
{
	int i;
	double dHomeVel;
	CanMsg msg;
	TimeStamp tsStart, tsNow;
	TimeStamp tsInit, tsLowHigh, tsHighLow;
	double dBlockWidth[20];
	double dBlockWidthSum[20];
	double dAngRad[20];
	FILE* pFile;

	LOGINFO ( "block width measurement started" );

	// set measurement velocity = homing velocity
	dHomeVel = m_DriveParam.getHomeVel();
	IntprtSetInt ( 8, 'J', 'V', 0, -10000, true );
	IntprtSetInt ( 4, 'B', 'G', 0, 0, true );

	// clear CAN buffer
	while ( m_pCanCtrl->receiveMsg ( &msg ) );

	// start measurement at input high!

	// find transition high - low
	waitOnDigIn ( 0, 1, 10 );
	tsInit.SetNow();
	LOGINFO ( "init found" );

	i = 0;
	do
	{
		// find transition low - high
		waitOnDigIn ( 1, 1, 10 );
		tsLowHigh.SetNow();

		// find transition high - low
		waitOnDigIn ( 0, 1, 10 );
		tsHighLow.SetNow();

		dBlockWidth[i] = tsHighLow - tsLowHigh;
		dBlockWidthSum[i] = tsHighLow - tsInit;

		LOGINFO ( i << " blockwidth:" << dBlockWidth[i] << " sum:" << dBlockWidthSum[i] );

		i++;
	}
	while ( i < 20 );


	//TODO:
	pFile = fopen ( "Arm/IniFiles/AbsBlockEnc.ini", "w" );

	fprintf ( pFile, "This file contains calibration data for block width homing\n" );
	fprintf ( pFile, "It is auto-generated, do not edit\n" );
	fprintf ( pFile, "\n[BlockWidth]\n" );
	for ( i = 0; i < 20; i++ )
	{
		fprintf ( pFile, "B%d=%.5lf\n", i, dBlockWidth[i] );
	}

	fprintf ( pFile, "\n[Angle]\n" );
	for ( i = 0; i < 20; i++ )
	{
		dAngRad[i] = m_DriveParam.getSign() * dBlockWidthSum[i] / dBlockWidthSum[19] * MathSup::TWO_PI;
		fprintf ( pFile, "A%d=%.5lf\n", i, dAngRad[i] );
	}

	fclose ( pFile );

	LOGINFO ( "calibration file written" );

	IntprtSetInt ( 8, 'J', 'V', 0, 0, true );
	IntprtSetInt ( 4, 'B', 'G', 0, 0, true );

	return true;
}
//-----------------------------------------------
bool CanDriveHarmonica::waitOnDigIn ( int iLevel, int iNumDigIn, double dTimeOutS )
{
	int iDigIn;
	CanMsg msg;
	TimeStamp tsStart, tsNow;

	tsStart.SetNow();
	do
	{
		IntprtSetInt ( 4, 'I', 'P', 0, 16, true );

		m_pCanCtrl->receiveMsgRetry ( &msg, 10 );

		if ( ( msg.getAt ( 0 ) == 'I' ) && ( msg.getAt ( 1 ) == 'P' ) )
		{
			iDigIn = 0x1FFFFF & (	( msg.getAt ( 7 ) << 24 ) | ( msg.getAt ( 6 ) << 16 ) |
			                      ( msg.getAt ( 5 ) << 8 ) | msg.getAt ( 4 ) );

			if ( ( iLevel == 1 ) & ( ( iDigIn & iNumDigIn ) != 0x00 ) )
			{
				return true;
			}

			if ( ( iLevel == 0 ) & ( ( iDigIn & iNumDigIn ) == 0x00 ) )
			{
				return true;
			}

			tsNow.SetNow();
			if ( ( tsNow - tsStart ) > dTimeOutS )
			{
				return false;
			}
		}
	}
	while ( true );

}


