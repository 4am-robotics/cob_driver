#include <cob_forcetorque/ForceTorqueCtrl.h>
#include <unistd.h>

ForceTorqueCtrl::ForceTorqueCtrl()
{
	out.open("force.txt"); 
}

ForceTorqueCtrl::~ForceTorqueCtrl()
{ 
}

bool ForceTorqueCtrl::Init()
{
	initCan();
	//SetActiveCalibrationMatrix(0);
	//ReadCalibrationMatrix();
}

void ForceTorqueCtrl::initCan()
{	
	std::cout << "initESDCan" << std::endl;
	m_Can = new CanESD("", false);
}

void ForceTorqueCtrl::ReadFTSerialNumber()
{	
	std::cout << "\n\n*********CheckCalMatrix**********" << std::endl;
	CanMsg CMsg;
	CMsg.setID(0x205);
	CMsg.setLength(0);

	bool ret = m_Can->transmitMsg(CMsg, true);

	CanMsg replyMsg;
	replyMsg.set(0,0);
	replyMsg.set(0,1);
	replyMsg.set(0,2);
	replyMsg.set(0,3);
	replyMsg.set(0,4);
	bool ret2 = m_Can->receiveMsg(&replyMsg);
	int length = replyMsg.getLength();
	std::cout << "reply ID: \t" << replyMsg.getID()<<std::endl;
	std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
	std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " 
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " " 
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " 
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;
}

void ForceTorqueCtrl::SetActiveCalibrationMatrix(int num)
{
	std::cout << "\n\n*******Setting Active Calibration Matrix Num to: "<< num <<"********"<< std::endl;
	BYTE b = 0;
	CanMsg CMsg;
	CMsg.setID(0x206);
	CMsg.setLength(1);
	CMsg.setAt(num,0);

	bool ret = m_Can->transmitMsg(CMsg, true);

	CanMsg replyMsg;
	bool ret2 = m_Can->receiveMsg(&replyMsg);
	if(ret2)
	{
		std::cout<<"reply ID: \t"<<replyMsg.getID()<<std::endl;
		std::cout<<"reply Length: \t"<<replyMsg.getLength()<<std::endl;
		if(replyMsg.getID() == 0x206)
		{
			std::cout<<"Setting Calibration Matrix succeed!"<<std::endl;
			std::cout<<"Calibration Matrix: "<<replyMsg.getAt(0)<<" is Activ!"<<std::endl;
		}
		else
			std::cout<<"Error: Received wrong opcode!"<<std::endl;
	}
	else
		std::cout<<"Error: Receiving Message failed!"<<std::endl;

}


void ForceTorqueCtrl::ReadCalibrationMatrix()
{
	Eigen::VectorXf vCoef(6);

	//Read Fx coefficients
	ReadMatrix(0, vCoef);
	m_v3FXGain = vCoef;

	//Read Fy coefficients
	ReadMatrix(1, vCoef);
	m_v3FYGain = vCoef;

	//Read Fz coefficients
	ReadMatrix(2, vCoef);
	m_v3FZGain = vCoef;

	//Read Tx coefficients
	ReadMatrix(3, vCoef);
	m_v3TXGain = vCoef;

	//Read Ty coefficients
	ReadMatrix(4, vCoef);
	m_v3TYGain = vCoef;

	//Read Tz coefficients
	ReadMatrix(5, vCoef);
	m_v3TZGain = vCoef;
	SetCalibMatrix();
}

void ForceTorqueCtrl::ReadMatrix(int axis, Eigen::VectorXf& vec)
{
	std::cout << "\n\n*******Read Matrix**********"<<std::endl;
	float statusCode = 0, sg0 = 0.0, sg1 = 0.0, sg2 = 0.0, sg3 = 0.0, sg4 = 0.0, sg5 = 0.0;

	CanMsg CMsg;
	CMsg.setID(0x202);
	CMsg.setLength(1);
	CMsg.setAt(axis,0);

	bool ret = m_Can->transmitMsg(CMsg, true);
	if(!ret)
	{
		std::cout<<"Error: Requesting Calibration Matrix!"<<std::endl;
		return;
	}

	CanMsg replyMsg;
	bool ret2 = m_Can->receiveMsg(&replyMsg);
	if(ret2)
	{
		std::cout << "reply ID: \t" << replyMsg.getID()<<std::endl;
		std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
		std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " 
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " " 
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " 
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;
		
		fbBuf.bytes[0] = replyMsg.getAt(3);
		fbBuf.bytes[1] = replyMsg.getAt(2);
		fbBuf.bytes[2] = replyMsg.getAt(1);
		fbBuf.bytes[3] = replyMsg.getAt(0);
		sg0 = fbBuf.value;
		
		
		fbBuf.bytes[0] = replyMsg.getAt(7);
		fbBuf.bytes[1] = replyMsg.getAt(6);
		fbBuf.bytes[2] = replyMsg.getAt(5);
		fbBuf.bytes[3] = replyMsg.getAt(4);
		sg1 = fbBuf.value;

	}
	else
		return;

	ret2 = m_Can->receiveMsg(&replyMsg);
	if(ret2)
	{
		std::cout << "reply ID: \t" << replyMsg.getID()<<std::endl;
		std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
		std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " 
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " " 
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " 
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;

		fbBuf.bytes[0] = replyMsg.getAt(3);
		fbBuf.bytes[1] = replyMsg.getAt(2);
		fbBuf.bytes[2] = replyMsg.getAt(1);
		fbBuf.bytes[3] = replyMsg.getAt(0);
		sg2 = fbBuf.value;
		
		fbBuf.bytes[0] = replyMsg.getAt(7);
		fbBuf.bytes[1] = replyMsg.getAt(6);
		fbBuf.bytes[2] = replyMsg.getAt(5);
		fbBuf.bytes[3] = replyMsg.getAt(4);
		sg3 = fbBuf.value;
	}
	else
		return;

	ret2 = m_Can->receiveMsg(&replyMsg);
	if(ret2)
	{
		std::cout << "reply ID: \t" << replyMsg.getID()<<std::endl;
		std::cout << "reply Length: \t" << replyMsg.getLength()<<std::endl;
		std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " 
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " " 
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " 
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;

		fbBuf.bytes[0] = replyMsg.getAt(3);
		fbBuf.bytes[1] = replyMsg.getAt(2);
		fbBuf.bytes[2] = replyMsg.getAt(1);
		fbBuf.bytes[3] = replyMsg.getAt(0);
		sg4 = fbBuf.value;
		
		fbBuf.bytes[0] = replyMsg.getAt(7);
		fbBuf.bytes[1] = replyMsg.getAt(6);
		fbBuf.bytes[2] = replyMsg.getAt(5);
		fbBuf.bytes[3] = replyMsg.getAt(4);
		sg5 = fbBuf.value;
	}
	else
		return;

	vec[0] = sg0; vec[1] = sg1; vec[2] = sg2; vec[3] = sg3; vec[4] = sg4; vec[5] = sg5;
	std::cout<<"Matix:  SG0: "<<sg0<<" SG1: "<<sg1<<" SG2: "<<sg2<<" SG3: "<<sg3<<" SG4: "<<sg4<<" SG5: "<<sg5<<std::endl;
}

void ForceTorqueCtrl::ReadFirmwareVersion()
{
	std::cout << "\n\n*******Reading Firmware Version: "<< std::endl;
	BYTE b = 0;
	CanMsg CMsg;
	CMsg.setID(0x20F);
	CMsg.setLength(0);

	bool ret = m_Can->transmitMsg(CMsg, true);

	CanMsg replyMsg;
	bool ret2 = m_Can->receiveMsg(&replyMsg);
	if(ret2)
	{
		std::cout<<"reply ID: \t"<<replyMsg.getID()<<std::endl;
		std::cout<<"reply Length: \t"<<replyMsg.getLength()<<std::endl;
		if(replyMsg.getID() == 0x20F)
		{
			std::cout<<"Reading Firmware Succeed!"<<std::endl;
			std::cout << "reply Data: \t" << replyMsg.getAt(0) << " " << replyMsg.getAt(1) << " " 
				      << replyMsg.getAt(2) << " " << replyMsg.getAt(3) << " " 
				      << replyMsg.getAt(4) << " " << replyMsg.getAt(5) << " " 
				      << replyMsg.getAt(6) << " " << replyMsg.getAt(7) << std::endl;
		}
		else
			std::cout<<"Error: Received wrong opcode!"<<std::endl;
	}
	else
		std::cout<<"Error: Receiving Message failed!"<<std::endl;
}

void ForceTorqueCtrl::ReadSGData(double &Fx, double &Fy, double &Fz, double &Tx, double &Ty, double &Tz)
{
	int statusCode = 0, sg0 = 0, sg1 = 0, sg2 = 0, sg3 = 0, sg4 = 0, sg5 = 0;

	CanMsg CMsg;
	CMsg.setID(0x200);
	CMsg.setLength(0);

	bool ret = m_Can->transmitMsg(CMsg, true);

	CanMsg replyMsg;
	bool ret2 = m_Can->receiveMsg(&replyMsg);
	unsigned char c[2];
	if(ret2)
	{
		int length = replyMsg.getLength();

		c[0] = replyMsg.getAt(0); //status code
		c[1] = replyMsg.getAt(1);
		//statusCode = (((char)c[0] << 8) | c[1]);
		
		c[0] = replyMsg.getAt(2); //sg0
		c[1] = replyMsg.getAt(3);

		//sg0 = (((char)c[0] << 8) | c[1]);
		sg0 = (short)((c[0] << 8) | c[1]);

		c[0] = replyMsg.getAt(4); //sg1
		c[1] = replyMsg.getAt(5);
		sg1 = (short)((c[0] << 8) | c[1]);

		c[0] = replyMsg.getAt(6); //sg2
		c[1] = replyMsg.getAt(7);
		sg2 = (short)((c[0] << 8) | c[1]);
	}
	else
		return;

	ret2 = m_Can->receiveMsg(&replyMsg);
	if(ret2)
	{
		int length = replyMsg.getLength();

		c[0] = replyMsg.getAt(0); //sg3
		c[1] = replyMsg.getAt(1);
		sg3 = (short)((c[0] << 8) | c[1]);

		c[0] = replyMsg.getAt(2); //sg4
		c[1] = replyMsg.getAt(3);
		sg4 = (short)((c[0] << 8) | c[1]);

		c[0] = replyMsg.getAt(4); //sg5
		c[1] = replyMsg.getAt(5);
		sg5 = (short)((c[0] << 8) | c[1]);
	}
	else
		return;


	//std::cout<<"\nsg0: "<<sg0<<" sg1: "<<sg1<<" sg2: "<<sg2<<" sg3: "<<sg3<<" sg4: "<<sg4<<" sg5: "<<sg5<<std::endl;
	//out<<"sg0: "<<sg0<<" sg1: "<<sg1<<" sg2: "<<sg2<<" sg3: "<<sg3<<" sg4: "<<sg4<<" sg5: "<<sg5<<std::endl;
	
	StrainGaugeToForce(sg0, sg1, sg2, sg3, sg4, sg5);
	
	Fx = m_vForceData[0]; Fy = m_vForceData[1]; Fz = m_vForceData[2]; 
	Tx = m_vForceData[3]; Ty= m_vForceData[4]; Tz = m_vForceData[5];
	//out<<"Fx: "<<Fx<<" Fy: "<<Fy<<" Fz: "<<Fz<<" Tx: "<<Tx<<" Ty: "<<Ty<<" Tz: "<<Tz<<std::endl;
	
}

void ForceTorqueCtrl::StrainGaugeToForce(int& sg0, int& sg1, int& sg2, int& sg3, int& sg4, int& sg5)
{
	Eigen::VectorXf v6SG(6);
	Eigen::VectorXf v6tmp(6);
	Eigen::VectorXf test(6);
	
	v6SG[0] = sg0; v6SG[1] = sg1; v6SG[2] = sg2; v6SG[3] = sg3; v6SG[4] = sg4; v6SG[5] = sg5;
	//v6SG[0] = 263; v6SG[1] = -272; v6SG[2] = -137; v6SG[3] = 9; v6SG[4] = 275; v6SG[5] = -258;

/*	
	v6tmp = v6SG - m_v3StrainGaigeOffset;
	//std::cout<<"\nv6tmp: \n"<< v6tmp <<std::endl;
	//std::cout<<"\nCalibration Matrix: \n"<<m_mXCalibMatrix<<"\n\n";
	//std::cout<<"\nCalibration Matrix Transposed: \n"<<m_mXCalibMatrix.transpose()<<"\n\n";

	test = m_mXCalibMatrix * v6tmp;
	//std::cout << "Test: \n" << test << std::endl;
	m_vForceData = test;
*/
	//std::cout<<"\nCalibration Matrix: \n"<<m_mXCalibMatrix<<"\n\n";
	//std::cout<<"Calibration Matrix: rows: "<<m_mXCalibMatrix.rows()<<" columns: "<<m_mXCalibMatrix.cols()<<std::endl;
	test = m_mXCalibMatrix * v6SG;
	//std::cout<<"test: rows: "<<test.rows()<<" columns: "<<test.cols()<<std::endl;
	//std::cout<<"v6SG: rows: "<<v6SG.rows()<<" columns: "<<v6SG.cols()<<std::endl;
	m_vForceData = test * 0.000001;
	//std::cout << "test: \n" << m_vForceData << std::endl;
	//std::cout<<"m_vForceData: rows: "<<m_vForceData.rows()<<" columns: "<<m_vForceData.cols()<<std::endl;
}

void ForceTorqueCtrl::SetGaugeOffset(float sg0Off, float sg1Off, float sg2Off, float sg3Off, float sg4Off, float sg5Off)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = sg0Off; tmp[1] = sg1Off; tmp[2] = sg2Off; tmp[3] = sg3Off; tmp[4] = sg4Off; tmp[5] = sg5Off; 
	m_v3StrainGaigeOffset = tmp;
	//std::cout<<"GaugeOffset: \n"<<m_v3StrainGaigeOffset<<"\n";
	//std::cout<<"GaugeOffset 0: \n"<<tmp[0]<<"\n\n";
}
void ForceTorqueCtrl::SetGaugeGain(float gg0, float gg1, float gg2, float gg3, float gg4, float gg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = gg0; tmp[1] = gg1; tmp[2] = gg2; tmp[3] = gg3; tmp[4] = gg4; tmp[5] = gg5; 
	m_v3GaugeGain = tmp;
	//std::cout<<"GaugeGain: \n"<<m_v3GaugeGain<<"\n\n";
}

void ForceTorqueCtrl::SetFXGain(float fxg0, float fxg1, float fxg2, float fxg3, float fxg4, float fxg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = fxg0; tmp[1] = fxg1; tmp[2] = fxg2; tmp[3] = fxg3; tmp[4] = fxg4; tmp[5] = fxg5; 
	m_v3FXGain = tmp;
	//std::cout<<"FXGain: \n"<<m_v3FXGain<<"\n\n";
}
void ForceTorqueCtrl::SetFYGain(float fyg0, float fyg1, float fyg2, float fyg3, float fyg4, float fyg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = fyg0; tmp[1] = fyg1; tmp[2] = fyg2; tmp[3] = fyg3; tmp[4] = fyg4; tmp[5] = fyg5; 
	m_v3FYGain = tmp;
	//std::cout<<"FYGain: \n"<<m_v3FYGain<<"\n\n";

}
void ForceTorqueCtrl::SetFZGain(float fzg0, float fzg1, float fzg2, float fzg3, float fzg4, float fzg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = fzg0; tmp[1] = fzg1; tmp[2] = fzg2; tmp[3] = fzg3; tmp[4] = fzg4; tmp[5] = fzg5; 
	m_v3FZGain = tmp;
	//std::cout<<"FZGain: \n"<<m_v3FZGain<<"\n\n";

}
void ForceTorqueCtrl::SetTXGain(float txg0, float txg1, float txg2, float txg3, float txg4, float txg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = txg0; tmp[1] = txg1; tmp[2] = txg2; tmp[3] = txg3; tmp[4] = txg4; tmp[5] = txg5; 
	m_v3TXGain = tmp;
	//std::cout<<"TXGain: \n"<<m_v3TXGain<<"\n\n";
}
void ForceTorqueCtrl::SetTYGain(float tyg0, float tyg1, float tyg2, float tyg3, float tyg4, float tyg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = tyg0; tmp[1] = tyg1; tmp[2] = tyg2; tmp[3] = tyg3; tmp[4] = tyg4; tmp[5] = tyg5; 
	m_v3TYGain = tmp;
	//std::cout<<"TYGain: \n"<<m_v3TYGain<<"\n\n";
}
void ForceTorqueCtrl::SetTZGain(float tzg0, float tzg1, float tzg2, float tzg3, float tzg4, float tzg5)
{
	Eigen::VectorXf tmp(6);
	tmp[0] = tzg0; tmp[1] = tzg1; tmp[2] = tzg2; tmp[3] = tzg3; tmp[4] = tzg4; tmp[5] = tzg5; 
	m_v3TZGain = tmp;
	//std::cout<<"TZGain: \n"<<m_v3TZGain<<"\n\n";
}

void ForceTorqueCtrl::CalcCalibMatrix()
{
	Eigen::MatrixXf tmp(6, 6);
	tmp[0] = m_v3FXGain[0]/m_v3GaugeGain[0];
	tmp[1] = m_v3FXGain[1]/m_v3GaugeGain[1];
	tmp[2] = m_v3FXGain[2]/m_v3GaugeGain[2];
	tmp[3] = m_v3FXGain[3]/m_v3GaugeGain[3];
	tmp[4] = m_v3FXGain[4]/m_v3GaugeGain[4];
	tmp[5] = m_v3FXGain[5]/m_v3GaugeGain[5];

	tmp[6] = m_v3FYGain[0]/m_v3GaugeGain[0];
	tmp[7] = m_v3FYGain[1]/m_v3GaugeGain[1];
	tmp[8] = m_v3FYGain[2]/m_v3GaugeGain[2];
	tmp[9] = m_v3FYGain[3]/m_v3GaugeGain[3];
	tmp[10] = m_v3FYGain[4]/m_v3GaugeGain[4];
	tmp[11] = m_v3FYGain[5]/m_v3GaugeGain[5];

	tmp[12] = m_v3FZGain[0]/m_v3GaugeGain[0];
	tmp[13] = m_v3FZGain[1]/m_v3GaugeGain[1];
	tmp[14] = m_v3FZGain[2]/m_v3GaugeGain[2];
	tmp[15] = m_v3FZGain[3]/m_v3GaugeGain[3];
	tmp[16] = m_v3FZGain[4]/m_v3GaugeGain[4];
	tmp[17] = m_v3FZGain[5]/m_v3GaugeGain[5];

	tmp[18] = m_v3TXGain[0]/m_v3GaugeGain[0];
	tmp[19] = m_v3TXGain[1]/m_v3GaugeGain[1];
	tmp[20] = m_v3TXGain[2]/m_v3GaugeGain[2];
	tmp[21] = m_v3TXGain[3]/m_v3GaugeGain[3];
	tmp[22] = m_v3TXGain[4]/m_v3GaugeGain[4];
	tmp[23] = m_v3TXGain[5]/m_v3GaugeGain[5];

	tmp[24] = m_v3TYGain[0]/m_v3GaugeGain[0];
	tmp[25] = m_v3TYGain[1]/m_v3GaugeGain[1];
	tmp[26] = m_v3TYGain[2]/m_v3GaugeGain[2];
	tmp[27] = m_v3TYGain[3]/m_v3GaugeGain[3];
	tmp[28] = m_v3TYGain[4]/m_v3GaugeGain[4];
	tmp[29] = m_v3TYGain[5]/m_v3GaugeGain[5];

	tmp[30] = m_v3TZGain[0]/m_v3GaugeGain[0];
	tmp[31] = m_v3TZGain[1]/m_v3GaugeGain[1];
	tmp[32] = m_v3TZGain[2]/m_v3GaugeGain[2];
	tmp[33] = m_v3TZGain[3]/m_v3GaugeGain[3];
	tmp[34] = m_v3TZGain[4]/m_v3GaugeGain[4];
	tmp[35] = m_v3TZGain[5]/m_v3GaugeGain[5];
	
	m_mXCalibMatrix = tmp;
			
}
void ForceTorqueCtrl::SetCalibMatrix()
{
	Eigen::MatrixXf tmp(6, 6);
	tmp[0] = m_v3FXGain[0];
	tmp[1] = m_v3FXGain[1];
	tmp[2] = m_v3FXGain[2];
	tmp[3] = m_v3FXGain[3];
	tmp[4] = m_v3FXGain[4];
	tmp[5] = m_v3FXGain[5];

	tmp[6] = m_v3FYGain[0];
	tmp[7] = m_v3FYGain[1];
	tmp[8] = m_v3FYGain[2];
	tmp[9] = m_v3FYGain[3];
	tmp[10] = m_v3FYGain[4];
	tmp[11] = m_v3FYGain[5];

	tmp[12] = m_v3FZGain[0];
	tmp[13] = m_v3FZGain[1];
	tmp[14] = m_v3FZGain[2];
	tmp[15] = m_v3FZGain[3];
	tmp[16] = m_v3FZGain[4];
	tmp[17] = m_v3FZGain[5];

	tmp[18] = m_v3TXGain[0];
	tmp[19] = m_v3TXGain[1];
	tmp[20] = m_v3TXGain[2];
	tmp[21] = m_v3TXGain[3];
	tmp[22] = m_v3TXGain[4];
	tmp[23] = m_v3TXGain[5];

	tmp[24] = m_v3TYGain[0];
	tmp[25] = m_v3TYGain[1];
	tmp[26] = m_v3TYGain[2];
	tmp[27] = m_v3TYGain[3];
	tmp[28] = m_v3TYGain[4];
	tmp[29] = m_v3TYGain[5];

	tmp[30] = m_v3TZGain[0];
	tmp[31] = m_v3TZGain[1];
	tmp[32] = m_v3TZGain[2];
	tmp[33] = m_v3TZGain[3];
	tmp[34] = m_v3TZGain[4];
	tmp[35] = m_v3TZGain[5];

	
	m_mXCalibMatrix = tmp.transpose();
			
}
