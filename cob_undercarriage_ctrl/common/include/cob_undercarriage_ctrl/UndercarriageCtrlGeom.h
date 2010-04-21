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

#ifndef UndercarriageCtrlGeom_INCLUDEDEF_H
#define UndercarriageCtrlGeom_INCLUDEDEF_H

#include <cob_utilities/IniFile.h>
#include <cob_utilities/MathSup.h>

class UndercarriageCtrlGeom
{
private:
	
	bool m_bEMStopActive;
	
	// Actual Values for PltfMovement (calculated from Actual Wheelspeeds)
	double m_dVelLongMMS;
	double m_dVelLatMMS;
	double m_dRotRobRadS;
	double m_dRotVelRadS;

	// Actual Wheelspeed (read from Motor-Ctrls)
	std::vector<double> m_vdVelGearDriveRadS;
	std::vector<double> m_vdVelGearSteerRadS;
	std::vector<double> m_vdDltAngGearDriveRad;
	std::vector<double> m_vdAngGearSteerRad;

	// Desired Pltf-Movement (set from PltfHwItf)
	double m_dCmdVelLongMMS;
	double m_dCmdVelLatMMS;
	double m_dCmdRotRobRadS;
	double m_dCmdRotVelRadS;

	// Interpolated Wheelspeeds (calculated from desired and current Pltf-Config)
	std::vector<double> m_vdVelGearDriveIntpRadS;
	std::vector<double> m_vdVelGearSteerIntpRadS;
	std::vector<double> m_vdAngGearSteerIntpRad;

	// Delta values for interpolating between two commands
	std::vector<double> m_vdDeltaAngIntpRad;
	std::vector<double> m_vdDeltaDriveIntpRadS;

	// Desired Wheelspeeds set to ELMO-Ctrl's (calculated from desired Pltf-Movement)
	std::vector<double> m_vdVelGearDriveCmdRadS;
	std::vector<double> m_vdVelGearSteerCmdRadS;
	std::vector<double> m_vdAngGearSteerCmdRad;

	// Target Wheelspeed and -angle (calculated from desired Pltf-Movement with Inverse without controle!)
	// This Values might not be valid (to high step response in steering rate, ...) for commanding the drives
	std::vector<double> m_vdAngGearSteerTarget1Rad; // alternativ 1 for steering angle
	std::vector<double> m_vdVelGearDriveTarget1RadS;
	std::vector<double> m_vdAngGearSteerTarget2Rad; // alternativ 2 for steering angle (+/- PI)
	std::vector<double> m_vdVelGearDriveTarget2RadS;

	/** Position of the Wheels' Steering Axis'
	 *  in cartesian (X/Y) and polar (Dist/Ang) coordinates
	 *  relative to robot coordinate System
	 */
	std::vector<double> m_vdWheelXPosMM;
	std::vector<double> m_vdWheelYPosMM;
	std::vector<double> m_vdWheelDistMM;
	std::vector<double> m_vdWheelAngRad;

	/** Exact Position of the Wheels' itself
	 *  in cartesian (X/Y) and polar (Dist/Ang) coordinates
	 *  relative to robot coordinate System
	 */
	std::vector<double> m_vdExWheelXPosMM;
	std::vector<double> m_vdExWheelYPosMM;
	std::vector<double> m_vdExWheelDistMM;
	std::vector<double> m_vdExWheelAngRad;

	struct ParamType
	{
		int iDistWheels;
		int iRadiusWheelMM;

		int iDistSteerAxisToDriveWheelMM;

		double dMaxDriveRateRadpS;
		double dMaxSteerRateRadpS;
		double dCmdRateS;
		std::vector<double> WheelNeutralPos;
		std::vector<double> vdSteerDriveCoupling;
		/** Factor between steering motion and steering induced motion of drive wheels
		 *  subtract from Drive-Wheel Vel to get effective Drive Velocity (Direct Kinematics)
		 *  add to Drive-Wheel Vel (Inverse Kinematics) to account for coupling when commanding velos
		 */
		std::vector<double> vdFactorVel;
	};

	ParamType m_UnderCarriagePrms;

	/** ------- Position Controller Steer Wheels -------
	 * Impedance-Ctrlr Prms
	 *  -> model Stiffness via Spring-Damper-Modell
	 *  -> only oriented at impedance-ctrl (no forces commanded)
	 *  m_dSpring	Spring-constant (elasticity)
	 *  m_dDamp		Damping coefficient (also prop. for Velocity Feedforward)
	 *  m_dVirtM	Virtual Mass of Spring-Damper System
	 *  m_dDPhiMax	maximum angular velocity (cut-off)
	 *  m_dDDPhiMax	maximum angular acceleration (cut-off)
	 */
	double m_dSpring, m_dDamp, m_dVirtM, m_dDPhiMax, m_dDDPhiMax;
	/** storage for internal controller states
	 *  m_vdCtrlVal is Vector with stored Controller-values of all wheels
	 *  m_vdCtrlVal[iWheelNr][iVariableNr]
	 *  iWheelNr: 		Index of Wheel Number (0..3)
	 *  iVariableNr:	0: previous Commanded deltaPhi e(k-1)
	 *					1: pre-previous Commanded deltaPhi e(k-2)
	 *					2: previous Commanded Velocity u(k-1)
	 */
	std::vector< std::vector<double> > m_vdCtrlVal;

	// Factor for thread cycle time of ThreadMotionPltfCtrl and ThreadUnderCarriageCtrl	
	double m_dThreadCycleMultiplier;
	
	// calculate inverse kinematics
	void CalcInverse(void);

	// calculate direct kinematics
	void CalcDirect(void);

	// calculate Exact Wheel Position in robot coordinates
	void CalcExWheelPos(void);

	// perform one discrete Controle Step
	void CalcControlStep(void);

public:

	// Constructor
	UndercarriageCtrlGeom(void);

	// Destructor
	~UndercarriageCtrlGeom(void);

	// Initialize Parameters for Controller and Kinematics
	void InitUndercarriageCtrl(void);

	// Set desired value for Plattform Velocity to UndercarriageCtrl (Sollwertvorgabe)
	void SetDesiredPltfVelocity(double dCmdVelLongMMS, double dCmdVelLatMMS, double dCmdRotRobRadS, double dCmdRotVelRadS);

	// Set actual values of wheels (steer/drive velocity/position) (Istwerte)
	void SetActualWheelValues(std::vector<double> vdVelGearDriveRadS, std::vector<double> vdVelGearSteerRadS, std::vector<double> vdDltAngGearDriveRad, std::vector<double> vdAngGearSteerRad);

	// Get result of inverse kinematics (without controller)
	void GetSteerDriveSetValues(std::vector<double> & vdVelGearDriveRadS, std::vector<double> & vdAngGearSteerRad);

	// Get set point values for the Wheels (including controller) from UndercarriangeCtrl
	void GetNewCtrlStateSteerDriveSetValues(std::vector<double> & vdVelGearDriveRadS, std::vector<double> & vdVelGearSteerRadS, std::vector<double> & vdAngGearSteerRad,
						double & dVelLongMMS, double & dVelLatMMS, double & dRotRobRadS, double & dRotVelRadS);

	// Get result of direct kinematics
	void GetActualPltfVelocity(double & dDeltaLongMM, double & dDeltaLatMM, double & dDeltaRotRobRad, double & dDeltaRotVelRad,
					double & dVelLongMMS, double & dVelLatMMS, double & dRotRobRadS, double & dRotVelRadS);

	// Set EM flag and stop Ctrlr
	void setEMStopActive(bool bEMStopActive);
	
	// operator overloading
	void operator=(const UndercarriageCtrlGeom & GeomCtrl);
};
#endif

