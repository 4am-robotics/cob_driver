#ifndef __ELMO_CTRL_H__
#define __ELMO_CTRL_H__

#include <canopen_motor/CanDriveItf.h>
#include <canopen_motor/CanDriveHarmonica.h>
#include <generic_can/CanItf.h>
#include <generic_can/CanPeakSys.h>
#include <generic_can/CanPeakSysUSB.h>

// Headers provided by cob-packages which should be avoided/removed^M
#include <cob3_utilities/IniFile.h>


//#include <Manipulation/Interfaces/armInterface.h>
//#include </CanDriveHarmonica.h>
//#include <Extern/Neobotix/Drivers/Can/CANPeakSysUSB.h>
//#include <Manipulation/ManipUtil/ManipulatorXML.h>
//#include <Extern/Neobotix/Drivers/Can/DriveParam.h>

#include <string>	
//#include <Manipulation/ManipUtil/Joint.h>


typedef struct _CanOpenAddress
{
	int TxPDO1;
	int TxPDO2;
	int RxPDO2;
	int TxSDO;
	int RxSDO;
} CanOpenAddress;

class ElmoCtrl;

typedef struct
{
	ElmoCtrl * pElmoCtrl;
} ElmoThreadArgs;


class ElmoCtrlParams
{

	public:
		ElmoCtrlParams();
		
		int Init(std::string CanDevice, int BaudRate, int ModuleID)
		{
			SetCanDevice(CanDevice);
			SetBaudRate(BaudRate);	
			SetModuleID(ModuleID);
			return 0;
		}
				
		//Can Device
		void SetCanDevice(std::string CanDevice){m_CanDevice = CanDevice;}
		std::string GetCanDevice(){return m_CanDevice;}
			
		//BaudRate
		void SetBaudRate(int BaudRate){m_BaudRate=BaudRate;}
		int GetBaudRate(){return m_BaudRate;}
	
		//ModuleIDs
		int GetModuleID(){return m_ModuleID;}
		void SetModuleID(int id){m_ModuleID = id;}
		
		//Angular Constraints
		void SetUpperLimit(double UpperLimit){m_UpperLimit = UpperLimit;}
		void SetLowerLimit(double LowerLimit){m_LowerLimit = LowerLimit;}
		void SetAngleOffset(double AngleOffset){m_Offset = AngleOffset;}
		double GetUpperLimit(){return m_UpperLimit;}
		double GetLowerLimit(){return m_LowerLimit;}
		double GetAngleOffset(){return m_Offset;}

		//HomingDir
		void SetHomingDir(int dir){m_HomingDir = dir;}
		int GetHomingDir(){return m_HomingDir;}

		//CanIniFile
		void SetCanIniFile(std::string iniFile){m_CanIniFile=iniFile;}
		std::string GetCanIniFile(){return m_CanIniFile;}
		
		
	private:
		int m_ModuleID;
		std::string  m_CanDevice;
		int m_BaudRate;
		int m_HomingDir;
		double m_Offset;
		double m_UpperLimit;
		double m_LowerLimit;
		std::string m_CanIniFile;

};

class ElmoCtrl
{

public:
	ElmoCtrl();
	~ElmoCtrl();

	bool Init(ElmoCtrlParams* params, bool home = true);


	double MoveJointSpace (std::vector<double>& Angle);
	

	bool Home();

	bool RecoverAfterEmergencyStop();

	void Stop();


	void setMaxVelocity(float radpersec)
	{
		m_MaxVel = radpersec;
	}

	double getConfig();

	double getJointVelocity();



	bool SetMotionCtrlType(int type);
	int GetMotionCtrlType(); 
	

	enum {
		POS_CTRL,
		VEL_CTRL
	} MOTION_CTRL_TYPE;


	CANPeakSysUSB* GetCanCtrl(){return m_CanCtrl;}
	bool m_ElmoCtrlThreadActive;
	/// @brief joint mutexes
	pthread_mutex_t   m_cs_elmoCtrlIO;
	CanDriveHarmonica * m_Joint;  

private:

	bool sendNetStartCanOpen(CanItf* canCtrl);

	int m_MotionCtrlType;	

	DriveParam * m_JointParams;

	int  m_CanBaseAddress;
	CanOpenAddress  m_CanAddress;

	CANPeakSysUSB * m_CanCtrl;

	float m_MaxVel;

	double  m_UpperLimit;
	double  m_LowerLimit;
	double  m_JointOffset;
	/*	
	Jointd* m_CurrentAngularVelocity;
	/// @brief current joint angles
	Jointd* m_CurrentJointAngles;
	*/
	/// @brief joint mutexes
	pthread_mutex_t   m_Angles_Mutex;
	/// @brief joint velocity mutexes
	pthread_mutex_t   m_AngularVel_Mutex;
	/// @brief  Thread IDs for PowerCube connection threads
	pthread_t   m_ElmoThreadID;
	/// @brief Arguments for Powercube connection Threads
	ElmoThreadArgs * m_ElmoCtrlThreadArgs;
	ElmoCtrlParams * m_Params;
	
	
	/// @brief the logfile for debugging info & errors:

};









#endif //__ELMO_CTRL_H__
