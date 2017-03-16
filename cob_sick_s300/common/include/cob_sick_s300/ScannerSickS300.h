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
 * ROS stack name: cob_driver
 * ROS package name: cob_sick_s300
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2009
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

#ifndef SCANNERSICKS300_INCLUDEDEF_H
#define SCANNERSICKS300_INCLUDEDEF_H
//-----------------------------------------------

// base classes
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <math.h>
#include <stdio.h>

#include <cob_sick_s300/SerialIO.h>
#include <cob_sick_s300/TelegramS300.h>

/**
 * Driver class for the laser scanner SICK S300 Professional.
 * This driver only supports use with 500KBaud in cont. mode
 *
 * if the scanner is in standby, the measurements are 0x4004 according to the Sick Support
 */

class ScannerSickS300
{
public:

	// set of parameters which are specific to the SickS300
	struct ParamType
	{
		int range_field; //measurement range (1 to 5) --> usually 1 (default)
		double dScale;		// scaling of the scan (multiply with to get scan in meters)
		double dStartAngle;	// scan start angle
		double dStopAngle;	// scan stop angle
	};

	// storage container for received scanner data
	struct ScanPolarType
	{
		double dr; // distance //r;
		double da; // angle //a;
		double di; // intensity; //bool bGlare;
	};

	enum
	{
		SCANNER_S300_READ_BUF_SIZE = 10000,
		READ_BUF_SIZE = 10000,
		WRITE_BUF_SIZE = 10000
	};

	// Constructor
	ScannerSickS300();

	// Destructor
	~ScannerSickS300();

	/**
	 * Opens serial port.
	 * @param pcPort used "COMx" or "/dev/tty1"
	 * @param iBaudRate baud rate
	 * @param iScanId the scanner id in the data header (7 by default)
	 */
	bool open(const char* pcPort, int iBaudRate, int iScanId);

	// not implemented
	void resetStartup();

	// not implmented
	void startScanner();

	// not implemented
	void stopScanner();
	//sick_lms.Uninitialize();

	// whether the scanner is currently in Standby or not
	bool isInStandby() {return m_bInStandby;}

	void purgeScanBuf();

	bool getScan(std::vector<double> &vdDistanceM, std::vector<double> &vdAngleRAD, std::vector<double> &vdIntensityAU, unsigned int &iTimestamp, unsigned int &iTimeNow, const bool debug);

	void setRangeField(const int field, const ParamType &param) {m_Params[field] = param;}

private:

	// Constants
	static const double c_dPi;

	// Parameters
	typedef std::map<int, ParamType> PARAM_MAP;
	PARAM_MAP m_Params;
	double m_dBaudMult;

	// Variables
	unsigned char m_ReadBuf[READ_BUF_SIZE+10];
	unsigned char m_ReadBuf2[READ_BUF_SIZE+10];
	unsigned int m_uiSumReadBytes;
	std::vector<int> m_viScanRaw;
	int m_iPosReadBuf2;
	static unsigned char m_iScanId;
	int m_actualBufferSize;
	bool m_bInStandby;

	// Components
	SerialIO m_SerialIO;
	TelegramParser tp_;

	// Functions
	void convertScanToPolar(const PARAM_MAP::const_iterator param, std::vector<int> viScanRaw,
							std::vector<ScanPolarType>& vecScanPolar);

};

//-----------------------------------------------
#endif
