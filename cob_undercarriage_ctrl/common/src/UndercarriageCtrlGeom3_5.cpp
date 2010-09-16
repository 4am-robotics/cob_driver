/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob3_drivers
 * ROS package name: cob_undercarriage_ctrl
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010:
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

#include <cob_undercarriage_ctrl/UndercarriageCtrlGeom.h>
#include <string>
#include <sstream>

FILE * pFile_WheelRadS;
FILE * pFile_SteerRad;

// Constructor
UndercarriageCtrlGeom::UndercarriageCtrlGeom(std::string sIniDirectory)
{
	m_sIniDirectory = sIniDirectory;

	// init EMStop flag
	m_bEMStopActive = false;
	IniFile iniFile;
	iniFile.SetFileName(m_sIniDirectory + "Platform.ini", "UnderCarriageCtrlGeom.cpp");
	iniFile.GetKeyInt("Config", "NumberOfWheels", &m_iNumberOfDrives, true);
	
	// init vectors
	m_vdVelGearDriveRadS.assign(m_iNumberOfDrives,0);
	m_vdVelGearSteerRadS.assign(m_iNumberOfDrives,0);
	m_vdDltAngGearDriveRad.assign(m_iNumberOfDrives,0);
	m_vdAngGearSteerRad.assign(m_iNumberOfDrives,0);

	/*
	m_vdVelGearDriveIntpRadS.assign(m_iNumberOfDrives,0);
	m_vdVelGearSteerIntpRadS.assign(m_iNumberOfDrives,0);
	m_vdAngGearSteerIntpRad.assign(m_iNumberOfDrives,0);
	*/
	
	m_vdVelGearDriveCmdRadS.assign(m_iNumberOfDrives,0);
	m_vdVelGearSteerCmdRadS.assign(m_iNumberOfDrives,0);
	m_vdAngGearSteerCmdRad.assign(m_iNumberOfDrives,0);

	m_vdWheelXPosMM.assign(m_iNumberOfDrives,0);
	m_vdWheelYPosMM.assign(m_iNumberOfDrives,0);
	m_vdWheelDistMM.assign(m_iNumberOfDrives,0);
	m_vdWheelAngRad.assign(m_iNumberOfDrives,0);

	m_vdExWheelXPosMM.assign(m_iNumberOfDrives,0);
	m_vdExWheelYPosMM.assign(m_iNumberOfDrives,0);
	m_vdExWheelDistMM.assign(m_iNumberOfDrives,0);
	m_vdExWheelAngRad.assign(m_iNumberOfDrives,0);

	m_vdAngGearSteerTarget1Rad.assign(m_iNumberOfDrives,0);
	m_vdVelGearDriveTarget1RadS.assign(m_iNumberOfDrives,0);
	m_vdAngGearSteerTarget2Rad.assign(m_iNumberOfDrives,0);
	m_vdVelGearDriveTarget2RadS.assign(m_iNumberOfDrives,0);
	m_vdAngGearSteerTargetRad.assign(m_iNumberOfDrives,0);
	m_vdVelGearDriveTargetRadS.assign(m_iNumberOfDrives,0);

	m_dCmdVelLongMMS = 0;
	m_dCmdVelLatMMS = 0;
	m_dCmdRotRobRadS = 0;
	m_dCmdRotVelRadS = 0;
	
	m_UnderCarriagePrms.WheelNeutralPos.assign(m_iNumberOfDrives,0);
	m_UnderCarriagePrms.vdSteerDriveCoupling.assign(m_iNumberOfDrives,0);
	m_UnderCarriagePrms.vdFactorVel.assign(m_iNumberOfDrives,0);
	
	m_vdCtrlVal.assign( m_iNumberOfDrives, std::vector<double> (2,0.0) );
	
	/*
	m_vdDeltaAngIntpRad.assign(m_iNumberOfDrives,0);
	m_vdDeltaDriveIntpRadS.assign(m_iNumberOfDrives,0);
	*/

	// init Prms of Impedance-Ctrlr
	m_dSpring = 10.0;
	m_dDamp = 2.5;
	m_dVirtM = 0.1;
	m_dDPhiMax = 12.0;
	m_dDDPhiMax = 100.0;
}

// Destructor
UndercarriageCtrlGeom::~UndercarriageCtrlGeom(void)
{

}

// Initialize Parameters for Controller and Kinematics
void UndercarriageCtrlGeom::InitUndercarriageCtrl(void)
{
	//LOG_OUT("Initializing Undercarriage-Controller (Geom)");
	IniFile iniFile;

	iniFile.SetFileName(m_sIniDirectory + "Platform.ini", "UnderCarriageCtrlGeom.cpp");
	iniFile.GetKeyInt("Geom", "DistWheels", &m_UnderCarriagePrms.iDistWheels, true);
	iniFile.GetKeyInt("Geom", "RadiusWheel", &m_UnderCarriagePrms.iRadiusWheelMM, true);
	iniFile.GetKeyInt("Geom", "DistSteerAxisToDriveWheelCenter", &m_UnderCarriagePrms.iDistSteerAxisToDriveWheelMM, true);

	iniFile.GetKeyDouble("Geom", "Wheel1XPos", &m_vdWheelXPosMM[0], true);
	iniFile.GetKeyDouble("Geom", "Wheel1YPos", &m_vdWheelYPosMM[0], true);

	iniFile.GetKeyDouble("DrivePrms", "MaxDriveRate", &m_UnderCarriagePrms.dMaxDriveRateRadpS, true);
	iniFile.GetKeyDouble("DrivePrms", "MaxSteerRate", &m_UnderCarriagePrms.dMaxSteerRateRadpS, true);

	iniFile.GetKeyDouble("DrivePrms", "Wheel1SteerDriveCoupling", &m_UnderCarriagePrms.vdSteerDriveCoupling[0], true);

	iniFile.GetKeyDouble("DrivePrms", "Wheel1NeutralPosition", &m_UnderCarriagePrms.WheelNeutralPos[0], true);

	for(int i = 0; i<m_iNumberOfDrives; i++)//for(int i = 0; i<4; i++)
	{	
		m_UnderCarriagePrms.WheelNeutralPos[i] = MathSup::convDegToRad(m_UnderCarriagePrms.WheelNeutralPos[i]);
		/*m_vdAngGearSteerIntpRad[i] = m_UnderCarriagePrms.WheelNeutralPos[i];*/
	}
	
	iniFile.GetKeyDouble("Thread", "ThrUCarrCycleTimeS", &m_UnderCarriagePrms.dCmdRateS, true);
	
	/*
	// calc cycle-time factor between Motion- and UCar-Thread for CMD-Interpolation
	m_dThreadCycleMultiplier = 2.0;	// default value
	double dThreadMotionPltf = 0.05;	
	iniFile.GetKeyDouble("Thread", "ThrMotionCycleTimeS", &dThreadMotionPltf, true);
	m_dThreadCycleMultiplier = dThreadMotionPltf/m_UnderCarriagePrms.dCmdRateS;
	m_dThreadCycleMultiplier = 1.0; // just for debug of ros node implementation
	*/

	// Read Values for Steering Position Controller from IniFile
	iniFile.SetFileName(m_sIniDirectory + "MotionCtrl.ini", "PltfHardwareCoB3.h");
	// Prms of Impedance-Ctrlr
	iniFile.GetKeyDouble("SteerCtrl", "Spring", &m_dSpring, true);
	iniFile.GetKeyDouble("SteerCtrl", "Damp", &m_dDamp, true);
	iniFile.GetKeyDouble("SteerCtrl", "VirtMass", &m_dVirtM, true);
	iniFile.GetKeyDouble("SteerCtrl", "DPhiMax", &m_dDPhiMax, true);
	iniFile.GetKeyDouble("SteerCtrl", "DDPhiMax", &m_dDDPhiMax, true);

	// calculate polar coords of Wheel Axis in robot coordinate frame
	for(int i = 0; i<m_iNumberOfDrives; i++)
	{
		m_vdWheelDistMM[i] = sqrt( (m_vdWheelXPosMM[i] * m_vdWheelXPosMM[i]) + (m_vdWheelYPosMM[i] * m_vdWheelYPosMM[i]) );
		m_vdWheelAngRad[i] = 0;//MathSup::atan4quad(m_vdWheelXPosMM[i], m_vdWheelYPosMM[i]);
	}

	pFile_WheelRadS = fopen ("Radgeschw_regler.txt","w");
	if (pFile_WheelRadS != NULL)
	{
		//fputs ("u(k) u(k-1) e(k) e(k-1)\n",pFile_WheelRadS);
		//fputs ("Drehrate Rad\n",pFile_WheelRadS);
	}

	pFile_SteerRad = fopen ("Lenkwinkel_regler.txt","w");
	if (pFile_SteerRad != NULL)
	{
		//fputs ("u(k) u(k-1) e(k) e(k-1)\n",pFile_SteerRad);
		//fputs ("Lenkwinkel\n",pFile_SteerRad);
	}
}

void UndercarriageCtrlGeom::CalcExWheelPos(void)
{
}

// Set desired value for Plattfrom Velocity to UndercarriageCtrl (Sollwertvorgabe)
void UndercarriageCtrlGeom::SetDesiredPltfVelocity(double dCmdVelLongMMS, double dCmdVelLatMMS, double dCmdRotRobRadS, double dCmdRotVelRadS)
{
	// copy function parameters to member variables
	m_dCmdVelLongMMS = dCmdVelLongMMS; // Sollwert geschwindigkeit in x richtung
	m_dCmdVelLatMMS = 0;
	m_dCmdRotRobRadS = dCmdRotRobRadS; // Sollwert rotation
	m_dCmdRotVelRadS = 0;

	std::cout << "Zeile 209 SetDesiredPltfVelocity" << "sollwert m_dCmdVelLongMMS " << m_dCmdVelLongMMS << std::endl;

	CalcInverse();
}

// Set actual values of wheels (steer/drive velocity/position) (Istwerte)
void UndercarriageCtrlGeom::SetActualWheelValues(std::vector<double> vdVelGearDriveRadS, std::vector<double> vdVelGearSteerRadS, std::vector<double> vdDltAngGearDriveRad, std::vector<double> vdAngGearSteerRad)
{
	//LOG_OUT("Set Wheel Position to Controller");

	m_vdVelGearDriveRadS = vdVelGearDriveRadS; // Istwert Drehrate Antriebsrad
	m_vdVelGearSteerRadS = vdVelGearSteerRadS; // Istwert Drehrate Lenkung
	m_vdDltAngGearDriveRad = vdDltAngGearDriveRad; // Istwert Winkel Antriebsrad
	m_vdAngGearSteerRad = vdAngGearSteerRad;	// Istwert Lenkwinkel
	
	// Peform calculation of direct kinematics (approx.) based on corrected Wheel Positions
	CalcDirect();
}

// Get result of inverse kinematics (without controller)
void UndercarriageCtrlGeom::GetSteerDriveSetValues(std::vector<double> & vdVelGearDriveRadS, std::vector<double> & vdAngGearSteerRad)
{

}

// Get set point values for the Wheels (including controller) from UndercarriangeCtrl
void UndercarriageCtrlGeom::GetNewCtrlStateSteerDriveSetValues(std::vector<double> & vdVelGearDriveRadS, std::vector<double> & vdVelGearSteerRadS, std::vector<double> & vdAngGearSteerRad,
								 double & dVelLongMMS, double & dVelLatMMS, double & dRotRobRadS, double & dRotVelRadS)
{
	// Einen Regelschritt durchführen mit:
	//	- Sollwerte Lenkwinkel und drehrate Rad, berechnet von CalcInverse()
	//		m_vdAngGearSteerTarget1Rad[i] = Sollwert Lenkwinkel
	//		m_vdVelGearDriveTarget1RadS[i] = Sollwert Drehrate Antriebsrad
	//	- Istwerte
	//		m_vdVelGearDriveRadS = Istwert Drehrate Antriebsrad
	//		m_vdVelGearSteerRadS = Istwert Drehrate Lenkung
	//		m_vdDltAngGearDriveRad = Istwert Winkel Antriebsrad
	//		m_vdAngGearSteerRad = Istwert Lenkwinkel
	//
	//	Als Ergebnis wird benötigt:
	//		wsteer und wdrive
	//		vdVelGearDriveRadS = Drehrate Antriebsrad
	//		vdVelGearSteerRadS = Drehrate Lenkung
	//		vdAngGearSteerRad = Lenkwinkel

	if(m_bEMStopActive == false)
	{
		//Calculate next step
		CalcControlStep();
	} else std::cout << "EM stop in undercar is active" << std::endl;

	vdVelGearDriveRadS = m_vdVelGearDriveCmdRadS;
	vdAngGearSteerRad = m_vdAngGearSteerCmdRad;
	vdVelGearSteerRadS = m_vdVelGearSteerCmdRadS;

	// die sind ohne bedeutung
	dVelLongMMS = m_dCmdVelLongMMS;
	dVelLatMMS = 0; //m_dCmdVelLatMMS;
	dRotRobRadS = m_dCmdRotRobRadS;
	dRotVelRadS = m_dCmdRotVelRadS;
}

// Get result of direct kinematics
void UndercarriageCtrlGeom::GetActualPltfVelocity(double & dDeltaLongMM, double & dDeltaLatMM, double & dDeltaRotRobRad, double & dDeltaRotVelRad,
												  double & dVelLongMMS, double & dVelLatMMS, double & dRotRobRadS, double & dRotVelRadS)
{
	dVelLongMMS = m_dVelLongMMS;
	dVelLatMMS = m_dVelLatMMS;
	dRotRobRadS = m_dRotRobRadS;
	dRotVelRadS = m_dRotVelRadS;

	// calculate travelled distance and angle (from velocity) for output
	dDeltaLongMM = dVelLongMMS * m_UnderCarriagePrms.dCmdRateS; //TODO hier müssen die richtigen zeiten rein
	dDeltaLatMM = dVelLatMMS * m_UnderCarriagePrms.dCmdRateS;
	dDeltaRotRobRad = dRotRobRadS * m_UnderCarriagePrms.dCmdRateS;
	dDeltaRotVelRad = dRotVelRadS * m_UnderCarriagePrms.dCmdRateS;
}

// calculate inverse kinematics
void UndercarriageCtrlGeom::CalcInverse(void)
{	
	// help variable to store velocities of the steering axis in mm/s
	double dtempAxVelXRobMMS, dtempAxVelYRobMMS;

	// check if zero movement commanded -> keep orientation of wheels, set wheel velocity to zero
	if((m_dCmdVelLongMMS == 0) && (m_dCmdVelLatMMS == 0) && (m_dCmdRotRobRadS == 0) && (m_dCmdRotVelRadS == 0))
	{
		for(int i = 0; i<m_iNumberOfDrives; i++)
		{
			m_vdAngGearSteerTarget1Rad[i] = m_vdAngGearSteerRad[i];
			m_vdVelGearDriveTarget1RadS[i] = 0;
			m_vdAngGearSteerTarget2Rad[i] = m_vdAngGearSteerRad[i];
			m_vdVelGearDriveTarget2RadS[i] = 0;
			
			//std::cout << "Returned CalcInverse due to zero command" << std::endl;
		}
		return;
	}

	// calculate sets of possible Steering Angle // Drive-Velocity combinations
	for(int i = 0; i<m_iNumberOfDrives; i++)//for(int i = 0; i<4; i++)
	{	
		// calculate velocity and direction of single wheel motion
		// Translational Portion
		dtempAxVelXRobMMS = m_dCmdVelLongMMS;
		dtempAxVelYRobMMS = 0;	

		// Rotational Portion
		dtempAxVelYRobMMS = m_dCmdRotRobRadS * m_vdWheelXPosMM[0];

		std::cout << "368 CalcInverse " << " dtempAxVelXRobMMS " << dtempAxVelXRobMMS << std::endl;

		// calculate resulting steering angle 
		// Wheel has to move in direction of resulting velocity vector of steering axis 
		m_vdAngGearSteerTarget1Rad[i] = MathSup::atan4quad(dtempAxVelYRobMMS, dtempAxVelXRobMMS); // Sollwert Lenkwinkel

		// calculate corresponding angle in opposite direction (+180 degree)
		m_vdAngGearSteerTarget2Rad[i] = m_vdAngGearSteerTarget1Rad[i] + MathSup::PI;
		MathSup::normalizePi(m_vdAngGearSteerTarget2Rad[i]);
		
		// calculate absolute value of rotational rate of driving wheels in rad/s
		m_vdVelGearDriveTarget1RadS[i] = sqrt( (dtempAxVelXRobMMS * dtempAxVelXRobMMS) + (dtempAxVelYRobMMS * dtempAxVelYRobMMS) ) / (double)m_UnderCarriagePrms.iRadiusWheelMM;
		// now adapt to direction (forward/backward) of wheel
		m_vdVelGearDriveTarget2RadS[i] = - m_vdVelGearDriveTarget1RadS[i];// Sollwert Drehrate Antriebsrad
	}
}

// calculate direct kinematics
void UndercarriageCtrlGeom::CalcDirect(void)
{
	// declare auxilliary variables
	double dtempVelXRobMMS;		// Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
	double dtempVelYRobMMS;		// Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)
	double dtempRotRobRADPS;	// Robot-Rotation-Rate in rad/s (in Robot-Coordinateframe)
	double dtempDiffXMM;		// Difference in X-Coordinate of two wheels in mm
	double dtempDiffYMM;		// Difference in Y-Coordinate of two wheels in mm
	//double dtempRelPhiWheelsRAD;	// Angle between axis of two wheels w.r.t the X-Axis of the Robot-Coordinate-System in rad
	//double dtempRelDistWheelsMM;	// distance of two wheels in mm 
	double dtempRelPhiWheel1RAD;	// Steering Angle of (im math. pos. direction) first Wheel w.r.t. the linking axis of the two wheels
	//double dtempRelPhiWheel2RAD;	// Steering Angle of (im math. pos. direction) second Wheel w.r.t. the linking axis of the two wheels
	std::vector<double> vdtempVelWheelMMS(m_iNumberOfDrives);//std::vector<double> vdtempVelWheelMMS(4);	// Wheel-Velocities (all Wheels) in mm/s

	// initial values
	dtempVelXRobMMS = 0;			// Robot-Velocity in x-Direction (longitudinal) in mm/s (in Robot-Coordinateframe)
	dtempVelYRobMMS = 0;			// Robot-Velocity in y-Direction (lateral) in mm/s (in Robot-Coordinateframe)
	dtempRotRobRADPS = 0;


	// calculate corrected wheel velocities
	for(int i = 0; i<m_iNumberOfDrives; i++)//for(int i = 0; i<4; i++)
	{
		// calc effective Driving-Velocity
		//vdtempVelWheelMMS[i] = m_UnderCarriagePrms.iRadiusWheelMM * (m_vdVelGearDriveRadS[i] - m_UnderCarriagePrms.vdFactorVel[i]* m_vdVelGearSteerRadS[i]);
		vdtempVelWheelMMS[i] = m_UnderCarriagePrms.iRadiusWheelMM * m_vdVelGearDriveRadS[i];//compute wheel velocity
	}

/*	// calculate rotational rate of robot and current "virtual" axis between all wheels
	for(int i = 0; i<3; i++)
	{
		// calc Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
		dtempDiffXMM = m_vdExWheelXPosMM[i+1] - m_vdExWheelXPosMM[i];
		dtempDiffYMM = m_vdExWheelYPosMM[i+1] - m_vdExWheelYPosMM[i];
		dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
		dtempRelPhiWheelsRAD = MathSup::atan4quad( dtempDiffYMM, dtempDiffXMM );

		// transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
		dtempRelPhiWheel1RAD = m_vdAngGearSteerRad[i] - dtempRelPhiWheelsRAD;
		dtempRelPhiWheel2RAD = m_vdAngGearSteerRad[i+1] - dtempRelPhiWheelsRAD;

		dtempRotRobRADPS += (vdtempVelWheelMMS[i+1] * sin(dtempRelPhiWheel2RAD) - vdtempVelWheelMMS[i] * sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;
	}
*/
	// calculate last missing axis (between wheel 4 and 1)
	// calc. Parameters (Dist,Phi) of virtual linking axis of the two considered wheels
	//dtempDiffXMM = m_vdExWheelXPosMM[0] - m_vdExWheelXPosMM[3];
	//dtempDiffYMM = m_vdExWheelYPosMM[0] - m_vdExWheelYPosMM[3];
	//dtempRelDistWheelsMM = sqrt( dtempDiffXMM*dtempDiffXMM + dtempDiffYMM*dtempDiffYMM );
	//dtempRelPhiWheelsRAD = MathSup::atan4quad( dtempDiffYMM, dtempDiffXMM );

	// transform velocity of wheels into relative coordinate frame of linking axes -> subtract angles
	dtempRelPhiWheel1RAD = m_vdAngGearSteerRad[0];// - dtempRelPhiWheelsRAD;
	//dtempRelPhiWheel2RAD = m_vdAngGearSteerRad[0] - dtempRelPhiWheelsRAD;

	// close calculation of robots rotational velocity
	//dtempRotRobRADPS += (vdtempVelWheelMMS[0]*sin(dtempRelPhiWheel2RAD) - vdtempVelWheelMMS[3]*sin(dtempRelPhiWheel1RAD))/dtempRelDistWheelsMM;
	
	// calculate linear velocity of robot
	for(int i = 0; i<m_iNumberOfDrives; i++)//for(int i = 0; i<4; i++)
	{
		dtempVelXRobMMS += vdtempVelWheelMMS[i]*cos(m_vdAngGearSteerRad[i]);
		dtempVelYRobMMS += vdtempVelWheelMMS[i]*sin(m_vdAngGearSteerRad[i]);
	}
dtempRotRobRADPS = dtempVelYRobMMS / m_vdWheelXPosMM[0]; //omega = velocity/radius

	// assign rotational velocities for output
	m_dRotRobRadS = dtempRotRobRADPS; //m_dRotRobRadS = dtempRotRobRADPS/4;
	m_dRotVelRadS = 0; // currently not used to represent 3rd degree of freedom -> set to zero

	// assign linear velocity of robot for output
	m_dVelLongMMS = dtempVelXRobMMS; //m_dVelLongMMS = dtempVelXRobMMS/4;
	m_dVelLatMMS = 0; //m_dVelLatMMS = dtempVelYRobMMS/4;

}

// perform one discrete Control Step (controls steering angle)
void UndercarriageCtrlGeom::CalcControlStep(void)
{	
	std::cout << "CalcControlStep" << std::endl;

	// declare auxilliary variables
	double kP = 0.1;//0.08; //1365; //0.5; // P-Anteil
	double kI = 0.1; //0.776; // I-Anteil (1 - T/Tn)
	double kP1 = 0.1;
	double kI1 = 0;
	double u, e; // u(k), e(k)
	static double u1k_1=0, e1k_1=0; // u(k-1), e(k-1) für Antriebsrad
	static double u2k_1=0, e2k_1=0; // u(k-1), e(k-1) für Lenkwinkel
	static bool bDebug = false;
	static int counter = 0;
	int temp = 0;
	double dDeltaPhi1=0, dDeltaPhi2=0;
	
	
	// check if zero movement commanded -> keep orientation of wheels, set steer velocity to zero
	if ((m_dCmdVelLongMMS == 0) && (m_dCmdVelLatMMS == 0) && (m_dCmdRotRobRadS == 0) && (m_dCmdRotVelRadS == 0))
	{
		//std::cout << "Zero command!" << std::endl;
		m_vdVelGearDriveCmdRadS.assign(m_iNumberOfDrives,0.0);		// set velocity for drives to zero
		m_vdVelGearSteerCmdRadS.assign(m_iNumberOfDrives,0.0);		// set velocity for steers to zero
		
		u1k_1=0; u2k_1 = 0;
		e1k_1 = 0; e2k_1 = 0;
		
		return;
	}


//---------------------------------------------------------------------------------------------------
// Berechnung der drehrate Antriebsrad
//---------------------------------------------------------------------------------------------------
	// Berechnen der Regeldifferenz e(k)

	//e = fabs(m_vdVelGearDriveTarget1RadS[0]) - fabs(m_vdVelGearDriveRadS[0]); // Sollwert - Istwert
	e = fabs(m_vdVelGearDriveTarget1RadS[0]) - fabs(m_vdVelGearDriveRadS[0]);
	//if(fabs(e) < 0.1)
	//	e=0;

	u = u1k_1 + kP*e - kP*kI*e1k_1;

	if(bDebug == true)
	{
		char s1[20] = {0};
		sprintf(s1,"%i", counter);
		fputs(s1, pFile_WheelRadS);
		fputs(" ", pFile_WheelRadS);
		sprintf(s1,"%lf", u );
		fputs(s1, pFile_WheelRadS);
		fputs(" ", pFile_WheelRadS);
		sprintf(s1,"%lf", m_vdVelGearDriveTarget1RadS[0] );//sollwert in 3.spalte
		fputs(s1, pFile_WheelRadS);
		fputs(" ", pFile_WheelRadS);
		sprintf(s1,"%lf", m_vdVelGearDriveRadS[0] ); //istwert in 4. Spalte
		fputs(s1, pFile_WheelRadS);
		fputs(" ", pFile_WheelRadS);
		sprintf(s1,"%lf", e );		//regeldifferenz in 5. spalte
		fputs(s1, pFile_WheelRadS);
		fputs("\n", pFile_WheelRadS);
	}
	if((counter > 1000) && (bDebug == true))
	{
		fclose (pFile_WheelRadS);
		fclose (pFile_SteerRad);
		bDebug = false;
	}

/*	if(fabs(u) > 3) //m_UnderCarriagePrms.dMaxSteerRateRadpS)
		{
			if (u > 0)
				u = m_vdVelGearDriveTarget1RadS[0];//m_UnderCarriagePrms.dMaxSteerRateRadpS;
			else
				u = -m_vdVelGearDriveTarget1RadS[0];//m_UnderCarriagePrms.dMaxSteerRateRadpS;
		}
*/

	e1k_1 = e;
	// Speichern des Reglerschritts für drehrate Antriebsrad
	m_vdVelGearDriveCmdRadS[0] = u;
	u1k_1 = u;
	
//---------------------------------------------------------------------------------------------------
// Berechnung des Lenkwinkels
//---------------------------------------------------------------------------------------------------
	// Berechnen der Regeldifferenz e(k)
	e = m_vdAngGearSteerTarget1Rad[0] - m_vdAngGearSteerRad[0]; // Sollwert - Istwert
	MathSup::normalizePi(e);

	u = u2k_1 - kP1*e - kP1*kI1*e2k_1;
	MathSup::normalizePi(u);

	if(bDebug == true)
	{
		char s1[20] = {0};
		sprintf(s1,"%i", counter);
		fputs(s1, pFile_SteerRad);
		fputs(" ", pFile_SteerRad);
		sprintf(s1,"%lf", u ); // Regelerausgang 2.Spalte
		fputs(s1, pFile_SteerRad);
		fputs(" ", pFile_SteerRad);
		sprintf(s1,"%lf", m_vdAngGearSteerTarget1Rad[0] ); //Sollwert 3.Spalte
		fputs(s1, pFile_SteerRad);
		fputs(" ", pFile_SteerRad);
		sprintf(s1,"%lf", m_vdAngGearSteerRad[0] ); //Istwert 4.Spalte
		fputs(s1, pFile_SteerRad);
		fputs(" ", pFile_SteerRad);
		sprintf(s1,"%lf", e );		// Regeldifferenz 5.Spalte
		fputs(s1, pFile_SteerRad);
		fputs("\n", pFile_SteerRad);
	}

	u2k_1 = u;
	e2k_1 = e;
	// Speichern des Reglerschritts für Lenkwinkel
	m_vdAngGearSteerCmdRad[0] = u;

//---------------------------------------------------------------------------------------------------
// Berechnung der Lenkgeschwindigkeit
//---------------------------------------------------------------------------------------------------
	dDeltaPhi1 = m_vdAngGearSteerTarget1Rad[0] - m_vdAngGearSteerRad[0];
	dDeltaPhi2 = m_vdAngGearSteerTarget2Rad[0] - m_vdAngGearSteerRad[0];
	MathSup::normalizePi(dDeltaPhi1);
	MathSup::normalizePi(dDeltaPhi2);

	if((dDeltaPhi1 <= dDeltaPhi2))
		{
			temp = -1;
			//	btemp = false;
		}
		else
		{
			temp = 1;
			//btemp = false;
		}

	if(fabs(e2k_1) > 0.0175) //Lenkgeschw. nur berechnen wenn Winkeldifferenz ungleich 0
	{
		// Speichern des Reglerschritts für Lenkgeschw.
		if(fabs(e2k_1) > 0.3)
			m_vdVelGearSteerCmdRadS[0] = 3*temp;
		else
			m_vdVelGearSteerCmdRadS[0] = 0.3*temp;

		if(fabs(e) > 2)
		{
			m_vdVelGearDriveCmdRadS[0] = 0; //beim Lenken soll sich das rad nicht drehen
		}
	}
	else
	{
		m_vdVelGearSteerCmdRadS[0] = 0;
	}

	// Check if Steeringvelocity overgo maximum allowed rates.
/*	if(fabs(m_vdVelGearDriveCmdRadS[0]) > 3) //m_UnderCarriagePrms.dMaxSteerRateRadpS)
	{
		if (m_vdVelGearDriveCmdRadS[0] > 0)
			m_vdVelGearDriveCmdRadS[0] = 3;//m_UnderCarriagePrms.dMaxSteerRateRadpS;
		else
			m_vdVelGearDriveCmdRadS[0] = -3;//m_UnderCarriagePrms.dMaxSteerRateRadpS;
	}*/
	counter++;	//MathSup::normalizePi(dDeltaPhi);
	
	std::cout << "REGLER: DRIVE = " << u1k_1 << " Lenkwinkel: " << m_vdAngGearSteerTarget1Rad[0] << std::endl;
}



// operator overloading
void UndercarriageCtrlGeom::operator=(const UndercarriageCtrlGeom & GeomCtrl)
{
	// Actual Values for PltfMovement (calculated from Actual Wheelspeeds)
	m_dVelLongMMS = GeomCtrl.m_dVelLongMMS;
	m_dVelLatMMS = GeomCtrl.m_dVelLatMMS;
	m_dRotRobRadS = GeomCtrl.m_dRotRobRadS;
	m_dRotVelRadS = GeomCtrl.m_dRotVelRadS;

	// Actual Wheelspeed (read from Motor-Ctrls)
	m_vdVelGearDriveRadS = GeomCtrl.m_vdVelGearDriveRadS;
	m_vdVelGearSteerRadS = GeomCtrl.m_vdVelGearSteerRadS;
	m_vdDltAngGearDriveRad = GeomCtrl.m_vdDltAngGearDriveRad;
	m_vdAngGearSteerRad = GeomCtrl.m_vdAngGearSteerRad;

	// Desired Pltf-Movement (set from PltfHwItf)
	m_dCmdVelLongMMS = GeomCtrl.m_dCmdVelLongMMS;
	m_dCmdVelLatMMS = GeomCtrl.m_dCmdVelLatMMS;
	m_dCmdRotRobRadS = GeomCtrl.m_dCmdRotRobRadS;
	m_dCmdRotVelRadS = GeomCtrl.m_dCmdRotVelRadS;

	// Desired Wheelspeeds (calculated from desired ICM-configuration)
	m_vdVelGearDriveCmdRadS = GeomCtrl.m_vdVelGearDriveCmdRadS;
	m_vdVelGearSteerCmdRadS = GeomCtrl.m_vdVelGearSteerCmdRadS;
	m_vdAngGearSteerCmdRad = GeomCtrl.m_vdAngGearSteerCmdRad;

	// Target Wheelspeed and -angle (calculated from desired Pltf-Movement with Inverse without controle!)
	// alternativ 1 for steering angle
	m_vdAngGearSteerTarget1Rad = GeomCtrl.m_vdAngGearSteerTarget1Rad;
	m_vdVelGearDriveTarget1RadS = GeomCtrl.m_vdVelGearDriveTarget1RadS;
	// alternativ 2 for steering angle (+/- PI)
	m_vdAngGearSteerTarget2Rad = GeomCtrl.m_vdAngGearSteerTarget2Rad;
	m_vdVelGearDriveTarget2RadS = GeomCtrl.m_vdVelGearDriveTarget2RadS;

	// Position of the Wheels' Steering Axis'
	m_vdWheelXPosMM = GeomCtrl.m_vdWheelXPosMM;
	m_vdWheelYPosMM = GeomCtrl.m_vdWheelYPosMM;
	m_vdWheelDistMM = GeomCtrl.m_vdWheelDistMM;
	m_vdWheelAngRad = GeomCtrl.m_vdWheelAngRad;

	// Exact Position of the Wheels' itself
	m_vdExWheelXPosMM = GeomCtrl.m_vdExWheelXPosMM;
	m_vdExWheelYPosMM = GeomCtrl.m_vdExWheelYPosMM;
	m_vdExWheelDistMM = GeomCtrl.m_vdExWheelDistMM;
	m_vdExWheelAngRad = GeomCtrl.m_vdExWheelAngRad;

	// Prms
	m_UnderCarriagePrms = GeomCtrl.m_UnderCarriagePrms;

	// Position Controller Steer Wheels
	// Impedance-Ctrlr
	m_dSpring = GeomCtrl.m_dSpring;
	m_dDamp = GeomCtrl.m_dDamp;
	m_dVirtM = GeomCtrl.m_dDPhiMax;
	m_dDPhiMax = GeomCtrl.m_dDPhiMax;
	m_dDDPhiMax = GeomCtrl.m_dDDPhiMax;
	// Storage for internal controller states
	m_vdCtrlVal = GeomCtrl.m_vdCtrlVal;
}

// set EM Flag and stop ctrlr if active
void UndercarriageCtrlGeom::setEMStopActive(bool bEMStopActive)
{
	m_bEMStopActive = bEMStopActive;

	// if emergency stop reset ctrlr to zero
	if(m_bEMStopActive)
	{
		// Steermodules
		for (int i = 0; i<m_iNumberOfDrives; i++)//for(int i=0; i<; i++)
		{
			for(int j=0; j< 2; j++)
			{
				m_vdCtrlVal[i][j] = 0.0;
			}
		}
		// Outputs
		for (int i = 0; i<m_iNumberOfDrives; i++)//for(int i=0; i<4; i++)
		{
			m_vdVelGearDriveCmdRadS[i] = 0.0;
			m_vdVelGearSteerCmdRadS[i] = 0.0;
		}
	}

}
