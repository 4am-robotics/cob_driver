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

#include <cob_sick_s300/ScannerSickS300.h>

//-----------------------------------------------

typedef unsigned char BYTE;

const double ScannerSickS300::c_dPi = 3.14159265358979323846;

const unsigned char ScannerSickS300::c_StartBytes[10] = {0,0,0,0,0,0,0,0,255,7};

unsigned char ScannerSickS300::m_iScanId = 7;

const unsigned short ScannerSickS300::crc_LookUpTable[256]
	   = { 
	   0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 
	   0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 
	   0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 
	   0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 
	   0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 
	   0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 
	   0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 
	   0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 
	   0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 
	   0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 
	   0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 
	   0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 
	   0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 
	   0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 
	   0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 
	   0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 
	   0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 
	   0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 
	   0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 
	   0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 
	   0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 
	   0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 
	   0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 
	   0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 
	   0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 
	   0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 
	   0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 
	   0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 
	   0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 
	   0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 
	   0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 
	   0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0 
	 }; 

//-----------------------------------------------
ScannerSickS300::ScannerSickS300()
{
	m_Param.iDataLength = 1104;
	m_Param.iHeaderLength = 24;
	// scanner has a half degree resolution and a VoW of 270 degrees
	m_Param.iNumScanPoints = 541;
	m_Param.dScale = 0.01;
	m_Param.dStartAngle = -135.0/180.0*c_dPi;
	m_Param.dStopAngle = 135.0/180.0*c_dPi;

	// allows to set different Baud-Multipliers depending on used SerialIO-Card
	m_dBaudMult = 1.0;

	// init scan with zeros
	m_viScanRaw.assign(541, 0);
	m_iPosReadBuf2 = 0;

}


//-------------------------------------------
ScannerSickS300::~ScannerSickS300()
{
	m_SerialIO.close();
}


// ---------------------------------------------------------------------------
bool ScannerSickS300::open(const char* pcPort, int iBaudRate, int iScanId=7)
{
    int bRetSerial;
 
	// for Care-O-bot3 S300 is set fixed to 500kBaud
	if (iBaudRate != 500000)
		return false;

	// update scan id (id=8 for slave scanner, else 7)
	m_iScanId = iScanId;
	
	// initialize Serial Interface
	m_SerialIO.setBaudRate(iBaudRate);
	m_SerialIO.setDeviceName(pcPort);
	m_SerialIO.setBufferSize(READ_BUF_SIZE - 10 , WRITE_BUF_SIZE -10 );
	m_SerialIO.setHandshake(SerialIO::HS_NONE);
	m_SerialIO.setMultiplier(m_dBaudMult);
	bRetSerial = m_SerialIO.open();
	m_SerialIO.setTimeout(0.0);
	m_SerialIO.SetFormat(8, SerialIO::PA_NONE, SerialIO::SB_ONE);

    if(bRetSerial == 0)
    {
	    // Clears the read and transmit buffer.
	    m_iPosReadBuf2 = 0;
	    m_SerialIO.purge();
	    return true;
    }
    else
    {
        return false;
    }
}


//-------------------------------------------
void ScannerSickS300::purgeScanBuf()
{
	m_iPosReadBuf2 = 0;
	m_SerialIO.purge();
}


//-------------------------------------------
void ScannerSickS300::resetStartup()
{
}


//-------------------------------------------
void ScannerSickS300::startScanner()
{
}


//-------------------------------------------
void ScannerSickS300::stopScanner()
{
}


//-----------------------------------------------
bool ScannerSickS300::getScan(std::vector<double> &vdDistanceM, std::vector<double> &vdAngleRAD, std::vector<double> &vdIntensityAU, unsigned int &iTimestamp, unsigned int &iTimeNow)
{
	bool bRet = false;
	int i,j;
	int iNumRead;
	int iNumData;
	int iFirstByteOfHeader;
	int iFirstByteOfData;
	unsigned int iTelegramNumber;
	unsigned int uiReadCRC;
	unsigned int uiCalcCRC;
	std::vector<ScanPolarType> vecScanPolar;
	vecScanPolar.resize(m_Param.iNumScanPoints);

	iNumRead = m_SerialIO.readNonBlocking((char*)m_ReadBuf, SCANNER_S300_READ_BUF_SIZE-2);
	
	if( iNumRead < m_Param.iDataLength )
	{
		// not enough data in queue --> abort reading
	  	printf("Not enough data in queue, read data at slower rate!\n");
		return false;
	}
	
	// Try to find scan. Searching backwards in the receive queue.
	for(i=iNumRead-m_Param.iDataLength; i>=0; i--)
	{
		// parse through the telegram until header with correct scan id is found
		if (
				( m_ReadBuf[i+0] == c_StartBytes[0] ) &&
				( m_ReadBuf[i+1] == c_StartBytes[1] ) &&
				( m_ReadBuf[i+2] == c_StartBytes[2] ) &&
				( m_ReadBuf[i+3] == c_StartBytes[3] ) &&
				( m_ReadBuf[i+4] == c_StartBytes[4] ) &&
				( m_ReadBuf[i+5] == c_StartBytes[5] ) &&
		  		( m_ReadBuf[i+8] == c_StartBytes[8] ) &&
		  		( m_ReadBuf[i+9] == m_iScanId )
		 )

		{
			// ---- Start bytes found
			iFirstByteOfHeader = i;
			
			//extract time stamp from header:
			iTimestamp = (m_ReadBuf[i+17]<<24) | (m_ReadBuf[i+16]<<16) | (m_ReadBuf[i+15]<<8) |  (m_ReadBuf[i+14]);
			iTelegramNumber = (m_ReadBuf[i+19]<<8) |  (m_ReadBuf[i+18]);
			
			if(iNumRead-iFirstByteOfHeader > m_Param.iDataLength+4+17) {
				/*
				Besides the actual data set we found some parts of the following message.
				This means we grabbed these during transmission of a new message, let's use that to sync ros time with sick time
				*/
				iTimeNow = (m_ReadBuf[i+m_Param.iDataLength+4+17]<<24) | (m_ReadBuf[i+m_Param.iDataLength+4+16]<<16) | (m_ReadBuf[i+m_Param.iDataLength+4+15]<<8) |  (m_ReadBuf[i+m_Param.iDataLength+4+14]);
			} else iTimeNow = 0;
			
			iFirstByteOfData = i + m_Param.iHeaderLength;
			
			// check length of transmitted data (see Telegram in .h for reference)
			// read out how many bytes were transmitted (every data package has two bytes)
			iNumData = 2 * getUnsignedWord(m_ReadBuf[iFirstByteOfHeader + 6],
										 m_ReadBuf[iFirstByteOfHeader + 7]);
			// if the Data does not correspond to the expected amount --> abort the reading process
			if ( iNumData != m_Param.iDataLength ) {
			  continue;
			}

			// check CRC
			// Telgramm includes "24 bytes Header" (4 byte Reply-Header + 20 Bytes Tel.-Header) + 
			// + 1082 bytes Data + 2 bytes CRC (last two bytes) (see h-file for more details)
			// --> iNumData (1104) total telegram bytes +4 Bytes from the reply header (--> +4)
			uiReadCRC = getUnsignedWord(m_ReadBuf[iFirstByteOfHeader + 4 + iNumData - 1],
										m_ReadBuf[iFirstByteOfHeader + 4 + iNumData - 2]);
			// calc CRC
			uiCalcCRC = createCRC(&m_ReadBuf[iFirstByteOfHeader + 4], m_Param.iDataLength - 2);

			// if CRC check is positive --> read out data
			if (uiReadCRC == uiCalcCRC)
			{
				for(j=0; j<m_Param.iNumScanPoints; j++)
				{
					// read data-words from the scan
					m_viScanRaw[j] = getUnsignedWord(m_ReadBuf[iFirstByteOfData + 2 * j + 1],
													 m_ReadBuf[iFirstByteOfData + 2 * j ]);
				}
				// Scan was succesfully read from buffer
				bRet = true;
				break;
			}
		}
	}
	
	if(bRet)
	{
		// convert data into range and intensity information
		convertScanToPolar(m_viScanRaw, vecScanPolar);

		// resize vectors to size of Scan
		vdDistanceM.resize(vecScanPolar.size());
		vdAngleRAD.resize(vecScanPolar.size());
		vdIntensityAU.resize(vecScanPolar.size());
		// assign outputs
		for(unsigned int i=0; i < vecScanPolar.size(); i++)
		{
			vdDistanceM[i] = vecScanPolar[i].dr;
			vdAngleRAD[i] = vecScanPolar[i].da;
			vdIntensityAU[i] = vecScanPolar[i].di;
		}	
	}

	return bRet;
}


//-------------------------------------------
unsigned int ScannerSickS300::createCRC(unsigned char *ptrData, int Size)
{ 
	int CounterWord; 
	unsigned short CrcValue=0xFFFF;

	for (CounterWord = 0; CounterWord < Size; CounterWord++) 
	{ 
		CrcValue = (CrcValue << 8) ^ crc_LookUpTable[ (((BYTE)(CrcValue >> 8)) ^ *ptrData) ]; 
		ptrData++; 
	} 

	return (CrcValue); 
}


//-------------------------------------------
void ScannerSickS300::convertScanToPolar(std::vector<int> viScanRaw,
							std::vector<ScanPolarType>& vecScanPolar )
{	
	double dDist;
	double dAngle, dAngleStep;
	double dIntens;

	dAngleStep = fabs(m_Param.dStopAngle - m_Param.dStartAngle) / double(m_Param.iNumScanPoints - 1) ;
	
	for(int i=0; i<m_Param.iNumScanPoints; i++)
	{
		dDist = double ((viScanRaw[i] & 0x1FFF) * m_Param.dScale);

		dAngle = m_Param.dStartAngle + i*dAngleStep;
		dIntens = double(viScanRaw[i] & 0x2000);

		vecScanPolar[i].dr = dDist;
		vecScanPolar[i].da = dAngle;
		vecScanPolar[i].di = dIntens;
	}
}
