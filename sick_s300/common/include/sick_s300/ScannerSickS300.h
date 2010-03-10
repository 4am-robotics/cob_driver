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
 * ROS stack name: cob3_driver
 * ROS package name: sick_s300
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
#include <iostream>
#include <math.h>

#include <sick_s300/SerialIO.h>

/** 
 * Driver class for the laser scanner SICK S300 Professional.
 * This driver only supports use with 500KBaud in cont. mode
 *
 * S300 header format in continuous mode:
 *	   
 *      | 00 00 00 00 |   4 byte reply header
 *
 *		Now starts the actual telegram
 *
 *      | 00 00 |         data block number (fixed)
 *      | xx xx |         size of data telegram (should be dec 1104)
 *      | FF 07 |         fixed
 *      | xx xx |         protocol version
 *      | 0x 00 |         status: 00 00 = normal, 01 00 = lockout
 *      | xx xx xx xx |   scan number
 *      | xx xx |         telegram number
 *      | BB BB |         fixed
 *      | 11 11 |         fixed
 *	       ...            data
 *      | xx xx |         CRC
 *	   
 * 	   Readout of buffer starts with Reply-Header
 *	   Reply-Header:byte 0 to 3 = 4 bytes
 *	   Telegram (as it is stored in the Buffer):	Position in the telegram
 *	   Header: 		bytes 4 to 23 = 20 bytes		bytes 0 to 19
 *	   Data:   		bytes 24 to 1105 = 1082 bytes	bytes 20 to 1101
 *	   CRC:    		bytes 1106, 1107 = 2 bytes		bytes 1102, 1103
 * 
 *	   for easier parsing Reply-Header and Telegram-Header are combined in the folllowing
 *	   --> Headerlength = 24 bytes (iHeaderLength)
 *	   --> Total length in buffer is 1108 bytes
 *	   --> Telegram length (read from telegram) is 1104 bytes (iDataLength)
 */

class ScannerSickS300
{
public:

	// set of parameters which are specific to the SickS300
	struct ParamType
	{
		int iDataLength;	// length of data telegram
		int iHeaderLength;	// length of telegram header
		int iNumScanPoints;	// number of measurements in the scan
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
	 */
	bool open(const char* pcPort, int iBaudRate);
	//bool open(char* pcPort, int iBaudRate);

	// not implemented
	void resetStartup();

	// not implmented
	void startScanner();

	// not implemented
	void stopScanner();
	//sick_lms.Uninitialize();

	void purgeScanBuf();

	bool getScan(std::vector<double> &vdDistanceM, std::vector<double> &vdAngleRAD, std::vector<double> &vdIntensityAU);
	//sick_lms.GetSickScan(values, num_values);

	// add sick_lms.GetSickScanResolution();

	// add sick_lms.GetSickMeasuringUnits();


private:

	// Constants
	static const unsigned short crc_LookUpTable[256];
	static const unsigned char c_StartBytes[10];
	static const double c_dPi;

	// Parameters
	ParamType m_Param;
	double m_dBaudMult;

	// Variables
	unsigned char m_ReadBuf[READ_BUF_SIZE+10];
	unsigned char m_ReadBuf2[READ_BUF_SIZE+10];
	unsigned int m_uiSumReadBytes;
	std::vector<int> m_viScanRaw;
	int m_iPosReadBuf2;

	// Components
	SerialIO m_SerialIO;

	// Functions
	unsigned int getUnsignedWord(unsigned char msb, unsigned char lsb)
	{
		return (msb << 8) | lsb;
	}

	unsigned int createCRC(unsigned char *ptrData, int Size);

	void convertScanToPolar(std::vector<int> viScanRaw,
							std::vector<ScanPolarType>& vecScanPolar);

};

//-----------------------------------------------
#endif
