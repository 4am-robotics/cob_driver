#ifndef FORCETORQUE
#define FORCETORQUE

#include <iostream>

#include "CAN/CanESD.h"
#include <Eigen/Core>
#include <Eigen/Array>
#include <fstream>

//opCodes for the ForceTorque Can Interface
//set as Can Message ID
#define READ_SG 	0x200
#define READ_MATRIX 	0x202
#define READ_CALIB	0x205
#define SET_CALIB	0x206
#define SET_BASEID	0x213
#define SET_BAUD	0x214
#define READ_FIRMWARE	0x215

class ForceTorqueCtrl
{
	public:	
		ForceTorqueCtrl();
		~ForceTorqueCtrl();

		bool Init();
		void ReadFTSerialNumber();
		void SetActiveCalibrationMatrix(int num);
		void ReadSGData(double &Fx, double &Fy, double &Fz, double &Tx, double &Ty, double &Tz);
		void ReadFirmwareVersion();
		void ReadCalibrationMatrix();

		void SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off);
		void SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5);
		void SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5);
		void SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5);
		void SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5);
		void SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5);
		void SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5);
		void SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5);

		void SetCalibMatrix();
		void CalcCalibMatrix();
		void StrainGaugeToForce(int& sg0, int& sg1, int& sg2, int& sg3, int& sg4, int& sg5);
	
	protected:
		void initCan();
		
	private:
		CanMsg CMsg;
		CanItf* m_Can;
		unsigned int d_len;
		Eigen::VectorXf m_v3StrainGaigeOffset;
		Eigen::VectorXf m_v3GaugeGain;
		Eigen::VectorXf m_v3FXGain;
		Eigen::VectorXf m_v3FYGain;
		Eigen::VectorXf m_v3FZGain;
		Eigen::VectorXf m_v3TXGain;
		Eigen::VectorXf m_v3TYGain;
		Eigen::VectorXf m_v3TZGain;
		Eigen::MatrixXf m_mXCalibMatrix;
		Eigen::MatrixXf m_vForceData;
		
		

		// the Parameter indicates the Axis row to Read
		// Fx = 0 | Fy = 1 | Fz = 2 | Tx = 3 | Ty = 4 | Tz = 5
		void ReadMatrix(int axis, Eigen::VectorXf& vec);
		

		union
		{
		    char bytes[2];
		    short int value;
		} ibBuf;

		union
		{
			char bytes[4];
			float value;
		} fbBuf;

		std::ofstream out; 
};

#endif
