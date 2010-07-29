#include <cob_camera_axis/ElmoCtrl.h>

//#include <Neobotix/Drivers/Can/CANPeakSysUSB.h>
//#include <canopen_motor/CanDummy.h>


/*
#include "Neobotix/Utilities/LogApp.h"
#include "Extern/Neobotix/Compat.h"
#include "Extern/Neobotix/Drivers/Can/CanItf.h"
*/
#include <iostream>

using namespace std;
//void * ElmoThreadRoutine(void* threadArgs);


void Sleep(int msecs){usleep(1000*msecs);}

bool ElmoCtrl::sendNetStartCanOpen(CanItf* canCtrl)
{
	bool ret = false;	

	CanMsg msg;

	msg.m_iID  = 0;
	msg.m_iLen = 2;
	msg.set(1,0,0,0,0,0,0,0);
	ret = canCtrl->transmitMsg(msg, false);

	usleep(100000);


	return ret;
}




ElmoCtrl::ElmoCtrl()
{
	m_Joint = NULL;
	m_JointParams = NULL;
	m_CanCtrl = NULL;
	m_CanBaseAddress = NULL;
	m_MotionCtrlType = POS_CTRL;
	m_MaxVel = 2.0;
	m_Params = NULL;
}
ElmoCtrl::~ElmoCtrl()
{


	if (m_Joint)
		delete m_Joint;
	if (m_JointParams)
		delete m_JointParams;
	if (m_CanCtrl)
		delete m_CanCtrl;

}

bool ElmoCtrl::Home()
{
	bool success = false;
	if (m_Joint != NULL) {
			m_Joint->initHoming();
			
			// You have to overwrite the trigger channel for Homing-event
			// iHomeEvent = 5 : event according to defined FLS switch (for scara arm)
			// iHomeEvent = 9 : event according to definded DIN1 switch (for full steerable wheels COb3)
			// TO DO: via param or .ini
			m_Joint->IntprtSetInt(8, 'H', 'M', 3, 5); 
			usleep(20000);
	}

	//ToDo: UHR: necessary?
	Sleep(10);
	int HomingDir = m_Params->GetHomingDir();
	printf("ElmoCtrl: Home(): Homing Vel = %f\n",HomingDir*0.3);
	m_Joint->setGearVelRadS(HomingDir*0.3);
	//ToDo: UHR: necessary?
	Sleep(750);
	success =m_Joint->execHoming();
	m_Joint->setGearVelRadS(0.0);

	return success;


}


int ElmoCtrl::evalCanBuffer()
{
	bool bRet;
	
	//pthread_mutex_lock(&(m_Mutex));

	// as long as there is something in the can buffer -> read out next message
	while(m_CanCtrl->receiveMsg(&m_CanMsgRec) == true)
	{
		bRet = false;
		// check for every motor if message belongs to it
		bRet |= m_Joint->evalReceivedMsg(m_CanMsgRec);

		if (bRet == true)
		{
		}
			
	};
	

	//pthread_mutex_unlock(&(m_Mutex));

	return 0;
}

bool ElmoCtrl::RecoverAfterEmergencyStop()
{
	
	bool success = false;
	printf("Resetting motor ...\n");
	success = m_Joint->start();
	if (!success)
	{
			printf("failed!\n");
	}
	else
	{	
			printf("successful\n");
			m_Joint->setGearVelRadS(0);
	}
	Sleep(1000);
	return success;

}


bool ElmoCtrl::Init(ElmoCtrlParams * params, bool home) //home = true by default
{
	bool success = false;
	
	string CanIniFile;
	string CanDevice;
	int baudrate = 0;
	
	m_Params = params;

	if (params == NULL)
	{
		printf("ElmoCtrlParams:Error:%s:%d:, invalid parameters!\n",__FILE__,__LINE__);
		success = false;
	}
	else
	{
		success = true;
	}

	if (success)
	{

		printf( "------------ ElmoCtrl Init ---------------\n");
		
		//Allocate memory
		m_Joint = new CanDriveHarmonica();
		m_JointParams = new DriveParam();
		m_CanBaseAddress = params->GetModuleID();
		CanIniFile = params->GetCanIniFile();	
		m_MaxVel = params->GetMaxVel();
		m_HomingDir = params->GetHomingDir();
		
		if (CanIniFile.length() == 0)
		{	
			printf("%s,%d:Error: Parameter 'CanIniFile' not given!\n",__FILE__,__LINE__);
			success = false;
		}
		CanDevice = params->GetCanDevice();
		if (CanDevice.length() == 0)
		{	
			printf("%s,%d:Error: Parameter 'Can-Device' not given!\n",__FILE__,__LINE__);
			success = false;
		}
		baudrate = params->GetBaudRate();
		if (baudrate == 0)
		{	
			printf("%s,%d:Error: Parameter 'Baud-Rate' not given!\n",__FILE__,__LINE__);
			success = false;
		}
		
		if (success)
		{
			m_JointOffset = params->GetAngleOffset();
			m_UpperLimit = params->GetUpperLimit();
			m_LowerLimit = params->GetLowerLimit();
		}


		if (success)
		{
			printf("The following parameters were successfully read from the parameter server (given through *params): \n");
			printf("CanIniFile: 	%s\n",CanIniFile.c_str());
			printf("CanDEvice: 	%s\n",CanDevice.c_str());
			printf("Baudrate: 	%d\n",baudrate);
			printf("Module ID: %d\n",m_CanBaseAddress);
			printf("Max Vel: %f\n",m_MaxVel);
			printf("Homing Dir: %d\n",m_HomingDir);
			printf("Offset/Limit(min/max)  %f/(%f,%f)\n",m_JointOffset,m_LowerLimit,m_UpperLimit);
		}
	}
	if (success)
	{
		m_CanCtrl = new CanESD(CanIniFile.c_str(), false);
		//m_CanCtrl = new CANPeakSysUSB(CanIniFile.c_str());
		if (m_CanCtrl == NULL)
		{
			printf("%s,%d:Error: Could not open Can Device!\n",__FILE__,__LINE__);
			success = false;
		}
	  }


	  
	
	  if (success)
	  {
		  
		  	//m_CanBaseAddress = params->GetModulID(i);
			m_CanAddress.TxPDO1 = 0x181 + m_CanBaseAddress -1;
			m_CanAddress.TxPDO2 = 0x285 + m_CanBaseAddress -1;
			m_CanAddress.RxPDO2 = 0x301 + m_CanBaseAddress -1;
			m_CanAddress.TxSDO = 0x491 + m_CanBaseAddress -1;
			m_CanAddress.RxSDO = 0x511 + m_CanBaseAddress -1;
			m_Joint->setCanItf(m_CanCtrl);
			m_Joint->setCanOpenParam(m_CanAddress.TxPDO1, 
						    m_CanAddress.TxPDO2, 
						    m_CanAddress.RxPDO2, 
						    m_CanAddress.TxSDO, 
						    m_CanAddress.RxSDO );
			printf("CanAdresses set to %d (Base), %x, %x, %x, %x, %x...\n", m_CanBaseAddress,
																		m_CanAddress.TxPDO1,
																		m_CanAddress.TxPDO2,
																		m_CanAddress.TxSDO,
																		m_CanAddress.RxSDO,
																		m_CanAddress.RxSDO);
		
	  }
	  if (success)
	  {
	  	success = sendNetStartCanOpen(m_CanCtrl);
	  }
	  if (success)
	  {
			  //ToDo: Read from File!
			  m_JointParams->setParam( //parameters taken from CanCtrl.ini
							  0, //int iDriveIdent,
							  4096,//int iEncIncrPerRevMot,
							  1,//double dVelMeasFrqHz,
							  1,//double dBeltRatio,
							  47.77,//double dGearRatio,
							  -1.0,//int iSign,
							  740000,//double dVelMaxEncIncrS,
							  1000000,//80000,//double dAccIncrS2,
							  1000000//80000//double dDecIncrS2),
					  //(int)m_JointOffsets[i], //iEncOffsetIncr
					  //false
					  );
			  m_Joint->setDriveParam(*m_JointParams);
	  }

	  if (success)
	  {
			  printf("Init motor ...\n");
			  success = m_Joint->init();
			  if (!success)
			  {
					  printf("failed!\n");
			  }
			  else
			  {	
					  printf("successful\n");
			  }
	  }
	  if (success)
			  success = SetMotionCtrlType(m_MotionCtrlType);
			  if(!success) std::cout << "Failed to SetMotionCtrlType to " << m_MotionCtrlType << std::endl;

	  if (success)
	  {
			  printf("Starting motor ..\n");
			  success = m_Joint->start();
			  if (!success)
			  {
					  printf("failed!\n");
			  }
			  else
			  {	
					  printf("successful\n");
					  m_Joint->setGearVelRadS(0);
			  }
	  }
	  //ToDo: UHR: necessary?
	  Sleep(1000);

	  if (success && home)
	  {
	  	std::cout << "Start homing procedure now.." << std::endl;
			  success = Home();
	  }
	  //Thread init
	  if (success)
	  {
			  pthread_mutex_init(&m_Mutex,NULL);	
			  //pthread_mutex_init(&m_AngularVel_Mutex,NULL);	
			  //pthread_mutex_init(&m_cs_elmoCtrlIO,NULL);	
			  //m_ElmoCtrlThreadArgs = new ElmoThreadArgs();
			  //m_ElmoCtrlThreadArgs->pElmoCtrl = this;
			  //m_CurrentAngularVelocity = new Jointd(m_DOF);
			  //m_CurrentJointAngles = new Jointd(m_DOF);
			  //m_ElmoCtrlThreadActive = true;
	
			/*
			  pthread_attr_t attr;
					  sched_param param;

					  pthread_attr_init(&attr);
					  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
					  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
					  pthread_attr_getschedparam(&attr, &param);
					  param.sched_priority = 7;
					  pthread_attr_setschedparam(&attr, &param);

			  pthread_create(&m_ElmoThreadID, &attr, ElmoThreadRoutine, (void*)(m_ElmoCtrlThreadArgs));
			  printf("ElmoCtrlThread started\n\n\n");
			*/

	  }



	  return success;
}
/*
   void * ElmoThreadRoutine(void* threadArgs)
   {
//get context
ElmoThreadArgs *args = (ElmoThreadArgs*)threadArgs;
ElmoCtrl* elmoCtrl = args->pElmoCtrl;
cout << "ElmoThreadRoutine running "<< endl;
while (elmoCtrl->m_ElmoCtrlThreadActive==true)
{
//get joints
Jointd jointsVel;
Jointd joints;


pthread_mutex_lock(&(elmoCtrl->m_cs_elmoCtrlIO));
millisleep(1);
CanMsg msg;
elmoCtrl->m_Joint->requestStatus();
//printf("request status %i\n", i);
Sleep(40);
while (elmoCtrl->GetCanCtrl()->receiveMsg(&msg))
{
//printf("ElmoThreadRoutine: Msg %02x received\n",msg.m_iID);
bool ret = false;
ret |= elmoCtrl->m_Joint->evalReceivedMsg(msg);
if (ret)
{
double pos = 0.0;
double vel = 0.0;
elmoCtrl->m_Joint->getGearPosVelRadS(&pos,&vel);
printf("ElmoThreadRoutine: Msg %d received, vel %f, pos %f\n",msg.m_iID,pos,vel);
joints.set(i, pos);
jointsVel.set(i, vel);
}

if (!ret)
{
//	printf("ElmoThreadRoutine: Warning, CanMsg received with unknown Address!\n");
}
else
{

//get current angles and angular velocities from ElmoCtrl via CAN

//printf( "PowerCubeThreadRoutine: Unlocking for CriticalSection ... ");
//printf("released\n");
elmoCtrl->SetCurrentJointAngles(joints);	
elmoCtrl->SetCurrentJointAngularVelocities(jointsVel);
}
}
pthread_mutex_unlock(&(elmoCtrl->m_cs_elmoCtrlIO));

}
return NULL;
}
 */
/*
   void ElmoCtrl::GetCurrentJointAngles( Jointd& joints )
   {

//std::cout << "setCurrentJoints: Waiting for Angular_Mutex ... "<<endl;
pthread_mutex_lock(&m_Angles_Mutex);
	joints = *(m_CurrentJointAngles);
	//std::cout << "setCurrentJoint:  Releasing Angular_Mutex ... "<<endl;
	pthread_mutex_unlock(&m_Angles_Mutex);
}
void ElmoCtrl::GetCurrentJointAngularVelocities( Jointd& joints  )
{
	//std::cout << "setCurrentJointVelocities: Waiting for Angular_Mutex ... "<<endl;
	pthread_mutex_lock(&m_AngularVel_Mutex);
	joints = *(m_CurrentAngularVelocity);
	//std::cout << "setCurrentJoint:  Releasing Angular_Mutex ... "<<endl;
	pthread_mutex_unlock(&m_AngularVel_Mutex);
}
void ElmoCtrl::SetCurrentJointAngularVelocities(const Jointd& joints )
{
	//std::cout << "setCurrentJointVelocities: Waiting for Angular_Mutex ... "<<endl;
	pthread_mutex_lock(&m_AngularVel_Mutex);

	*m_CurrentAngularVelocity = joints;

	//std::cout << "setCurrentJoint:  Releasing Angular_Mutex ... "<<endl;
	pthread_mutex_unlock(&m_AngularVel_Mutex);
}
void ElmoCtrl::SetCurrentJointAngles(const Jointd& joints )
{
	//std::cout << "setCurrentJoints "<<manipId<<": Waiting for Angular_Mutex ... "<<endl;
	pthread_mutex_lock(&(m_Angles_Mutex));

	*m_CurrentJointAngles = joints;

	//std::cout << "setCurrentJoint:  "<<manipId<<" Releasing Angular_Mutex ... "<<endl;
	pthread_mutex_unlock(&(m_Angles_Mutex));
}

/// @brief return the DOF of arm / neck:
*/
bool ElmoCtrl::SetMotionCtrlType(int type)
{
	m_MotionCtrlType = type;
	bool success = false; 
	if (type == POS_CTRL)
	{
			success = m_Joint->shutdown();
			if (success)
					success = m_Joint->setTypeMotion(CanDriveHarmonica::MOTIONTYPE_POSCTRL);
			//ToDo: necessary?
			Sleep(100);
			success = m_Joint->start();

	}
	else if (type == VEL_CTRL)
	{
		//UHR: ToDo
		printf("%s%d:Error: Velocity control not implemented yet!\n",__FILE__,__LINE__);
		success = false;
	}
	return success;
};


int ElmoCtrl::GetMotionCtrlType() 
{
	return m_MotionCtrlType;
}

int ElmoCtrl::getGearPosVelRadS( double* pdAngleGearRad, double* pdVelGearRadS)
{

	// init default outputs
	*pdAngleGearRad = 0;
	*pdVelGearRadS = 0;

	m_Joint->getGearPosVelRadS(pdAngleGearRad, pdVelGearRadS);
	*pdAngleGearRad -= m_JointOffset;
	
	m_Joint->requestPosVel();
	
	return 0;
}

//-----------------------------------------------

int ElmoCtrl:: setGearPosVelRadS(double dPosRad, double dVelRadS)
{		
	//pthread_mutex_lock(&(m_Mutex));		

	if(dPosRad< m_LowerLimit) {
		std::cout << "Position under LowerBound -> set up" << std::endl;
		dPosRad = m_LowerLimit;
	} else if(dPosRad > m_UpperLimit) {
		std::cout << "Position over UpperBound -> set down" << std::endl;
		dPosRad = m_UpperLimit;
	}
		
	if(dVelRadS > m_MaxVel)
		dVelRadS = m_MaxVel;
	else if(dVelRadS < -m_MaxVel)
		dVelRadS = -m_MaxVel;

	m_Joint->setGearPosVelRadS(dPosRad + m_JointOffset, dVelRadS);
	
	//pthread_mutex_unlock(&(m_Mutex));
	
	return 0;
}




bool ElmoCtrl::Stop()
{
		//UHR: ToDo: what happens exactly in this method? Sudden stop?
		double pos = 0.0;
		double vel = 0.0;
		m_Joint->getGearPosVelRadS(&pos,&vel);
		m_Joint->setGearPosVelRadS(pos,0);

		return true;
		//m_Joint[i]->shutdown();

}

/*
Jointd ElmoCtrl::getJointVelocities()
{
	Jointd currentVel(m_DOF);
	GetCurrentJointAngularVelocities(currentVel);

	return currentVel;
}

Jointd ElmoCtrl::getConfig()
{
	Jointd currentConfig(m_DOF);
	GetCurrentJointAngles(currentConfig);
	for (int i=0; i < m_DOF; i++)
	{
		double pos = 0.0;
		double vel = 0.0;
		m_Joint[i]->getGearPosVelRadS(&pos,&vel);
		currentConfig.set(i, pos);
		Sleep(100);
		//printf("Joint %d: current config/vel %f, %f\n",i,pos,vel);
	}
	


	return currentConfig;
}
*/









