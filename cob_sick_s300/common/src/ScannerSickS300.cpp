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

#include <stdint.h>
#include <arpa/inet.h>

//-----------------------------------------------

typedef unsigned char BYTE;

const double ScannerSickS300::c_dPi = 3.14159265358979323846;
unsigned char ScannerSickS300::m_iScanId = 7;

const unsigned short crc_LookUpTable[256]
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

class TelegramParser {

	#pragma pack(push,1)
	union TELEGRAM_COMMON {
		struct {
			uint32_t reply_telegram;
			uint16_t trigger_result;
			uint16_t size;
			uint8_t  coordination_flag;
			uint8_t  device_addresss;
			uint16_t protocol_version;
			uint16_t status;
			uint32_t scan_number;
			uint16_t telegram_number;
			uint16_t type;
		};
		uint8_t bytes[22];
	};

	union TELEGRAM_DISTANCE {
		struct {
			uint16_t type;
		};
		uint8_t bytes[2];
	};

	union TELEGRAM_TAIL {
		struct {
			uint16_t crc;
		};
		uint8_t bytes[2];
	};

	union TELEGRAM_S300_DIST_2B {
		struct {
			unsigned distance : 13; //cm
			unsigned bit13 : 1;  //reflector or scanner distorted
			unsigned protective : 1;
			unsigned warn_field : 1;
		};
		uint8_t bytes[2];
	};

	#pragma pack(pop)

	enum TELEGRAM_COMMON_HS {JUNK_SIZE=4};
	enum TELEGRAM_COMMON_TYPES {IO=0xAAAA, DISTANCE=0xBBBB, REFLEXION=0xCCCC};
	enum TELEGRAM_DIST_SECTOR {_1=0x1111, _2=0x2222, _3=0x3333, _4=0x4444, _5=0x5555};


	static void ntoh(TELEGRAM_COMMON &tc) {
		tc.reply_telegram = ntohl(tc.reply_telegram);
		tc.trigger_result = ntohs(tc.trigger_result);
		tc.size = ntohs(tc.size);
		tc.protocol_version = ntohs(tc.protocol_version);
		tc.status = ntohs(tc.status);
		tc.scan_number = ntohl(tc.scan_number);
		tc.telegram_number = ntohs(tc.telegram_number);
		tc.type = ntohs(tc.type);
	}

	static void ntoh(TELEGRAM_DISTANCE &tc) {
		tc.type = ntohs(tc.type);
	}

	static void ntoh(TELEGRAM_TAIL &tc) {
	    //crc calc. is also in network order
		//tc.crc = ntohs(tc.crc);
	}

	static void print(const TELEGRAM_COMMON &tc) {
		std::cout<<"HEADER"<<std::endl;
		std::cout<<"reply_telegram"<<":"<<tc.reply_telegram<<std::endl;
		std::cout<<"trigger_result"<<":"<<tc.trigger_result<<std::endl;
		std::cout<<"size"<<":"<<2*tc.size<<std::endl;
		std::cout<<"coordination_flag"<<":"<< std::hex<<tc.coordination_flag<<std::endl;
		std::cout<<"device_addresss"<<":"<< std::hex<<tc.device_addresss<<std::endl;
		std::cout<<"protocol_version"<<":"<< std::hex<<tc.protocol_version<<std::endl;
		std::cout<<"status"<<":"<<tc.status<<std::endl;
		std::cout<<"scan_number"<<":"<< std::hex<<tc.scan_number<<std::endl;
		std::cout<<"telegram_number"<<":"<< std::hex<<tc.telegram_number<<std::endl;
		std::cout<<"type"<<":"<< std::hex<<tc.type<<std::endl;
		switch(tc.type) {
			case IO: std::cout<<"type"<<": "<<"IO"<<std::endl; break;
			case DISTANCE: std::cout<<"type"<<": "<<"DISTANCE"<<std::endl; break;
			case REFLEXION: std::cout<<"type"<<": "<<"REFLEXION"<<std::endl; break;
			default: std::cout<<"type"<<": "<<"unknown "<<tc.type<<std::endl; break;
		}
		std::cout<<std::dec<<std::endl;
	}

	static void print(const TELEGRAM_DISTANCE &tc) {
		std::cout<<"DISTANCE"<<std::endl;
		std::cout<<"type"<<":"<< std::hex<<tc.type<<std::endl;
		switch(tc.type) {
			case _1: std::cout<<"field 1"<<std::endl; break;
			case _2: std::cout<<"field 2"<<std::endl; break;
			case _3: std::cout<<"field 3"<<std::endl; break;
			case _4: std::cout<<"field 4"<<std::endl; break;
			case _5: std::cout<<"field 5"<<std::endl; break;
			default: std::cout<<"unknown "<<tc.type<<std::endl; break;
		}
		std::cout<<std::dec<<std::endl;
	}

	static void print(const TELEGRAM_TAIL &tc) {
		std::cout<<"TAIL"<<std::endl;
		std::cout<<"crc"<<":"<< std::hex<<tc.crc<<std::endl;
		std::cout<<std::dec<<std::endl;
	}

	//-------------------------------------------
	static unsigned int createCRC(uint8_t *ptrData, int Size)
	{ 
		int CounterWord; 
		unsigned short CrcValue=0xFFFF;

		for (CounterWord = 0; CounterWord < Size; CounterWord++) 
		{ 
			CrcValue = (CrcValue << 8) ^ crc_LookUpTable[ (((uint8_t)(CrcValue >> 8)) ^ *ptrData) ]; 
			ptrData++; 
		} 

		return (CrcValue); 
	}

	//supports versions: 0301, 0201
	static bool check(const TELEGRAM_COMMON &tc, const uint8_t DEVICE_ADDR) {
		uint8_t TELEGRAM_COMMON_PATTERN_EQ[] = {0,0,0,0, 0,0, 0,0, 0xFF, DEVICE_ADDR, 2, 1};
		uint8_t TELEGRAM_COMMON_PATTERN_OR[] = {0,0,0,0, 0,0, 0xff,0xff, 0,0, 1, 0};

		for(size_t i=0; i<sizeof(TELEGRAM_COMMON_PATTERN_EQ); i++) {
			if(TELEGRAM_COMMON_PATTERN_EQ[i] != (tc.bytes[i]&(~TELEGRAM_COMMON_PATTERN_OR[i])) ) {
				//std::cout<<"invalid at byte "<<i<<std::endl;
				return false;
			}
		}

		return true;
	}

	TELEGRAM_COMMON tc_;
	TELEGRAM_DISTANCE td_;
public:

	bool parseHeader(const unsigned char *buffer, const size_t max_size, const uint8_t DEVICE_ADDR)
	{
		if(sizeof(tc_)>max_size) return false;
		tc_ = *((TELEGRAM_COMMON*)buffer);

		if(!check(tc_, DEVICE_ADDR))
			return false;
		ntoh(tc_);
		//print(tc_);

		if(tc_.size*2+JUNK_SIZE>(int)max_size) {std::cout<<"inv4"<<std::endl;return false;}

		TELEGRAM_TAIL tt = *((TELEGRAM_TAIL*) (buffer+(2*tc_.size+JUNK_SIZE-sizeof(TELEGRAM_TAIL))) );
		ntoh(tt);
		//print(tt);

		if(tt.crc!=createCRC((uint8_t*)buffer+JUNK_SIZE, 2*tc_.size-sizeof(TELEGRAM_TAIL)))
			return false;

		memset(&td_, 0, sizeof(td_));
		switch(tc_.type) {
			case IO: break;

			case DISTANCE:
				td_ = *((TELEGRAM_DISTANCE*)(buffer+sizeof(tc_)));
				ntoh(td_);
				//print(td_);
				break;

			case REFLEXION: break;
			default: return false;
		}

		return true;
	}

	bool isDist() const {return tc_.type==DISTANCE;}
	int getField() const {
		switch(td_.type) {
			case _1: return 0;
			case _2: return 1;
			case _3: return 2;
			case _4: return 3;
			case _5: return 4;
			default: return -1;
		}
	}

	void readDistRaw(const unsigned char *buffer, std::vector<int> &res) const
	{
		res.clear();
		if(!isDist()) return;

		size_t num_points = (2*tc_.size - (sizeof(tc_)+sizeof(td_)+sizeof(TELEGRAM_TAIL)-JUNK_SIZE));
		//std::cout<<"num_points: "<<num_points/sizeof(TELEGRAM_S300_DIST_2B)<<std::endl;
		size_t i=0;
		for(; i<num_points; ) {
			TELEGRAM_S300_DIST_2B dist = *((TELEGRAM_S300_DIST_2B*) (buffer+(sizeof(tc_)+sizeof(td_)+i)) );
			res.push_back((int)dist.distance);
			i += sizeof(TELEGRAM_S300_DIST_2B);
		}
	}

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
	
	m_actualBufferSize = 0;

}


//-------------------------------------------
ScannerSickS300::~ScannerSickS300()
{
	m_SerialIO.closeIO();
}


// ---------------------------------------------------------------------------
bool ScannerSickS300::open(const char* pcPort, int iBaudRate, int iScanId=7)
{
    int bRetSerial;

	// update scan id (id=8 for slave scanner, else 7)
	m_iScanId = iScanId;
	
	// initialize Serial Interface
	m_SerialIO.setBaudRate(iBaudRate);
	m_SerialIO.setDeviceName(pcPort);
	m_SerialIO.setBufferSize(READ_BUF_SIZE - 10 , WRITE_BUF_SIZE -10 );
	m_SerialIO.setHandshake(SerialIO::HS_NONE);
	m_SerialIO.setMultiplier(m_dBaudMult);
	bRetSerial = m_SerialIO.openIO();
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
	int i;
	int iNumRead = 0;
	int iNumRead2 = 0;
	std::vector<ScanPolarType> vecScanPolar;
	vecScanPolar.resize(m_Param.iNumScanPoints);

	iNumRead2 = m_SerialIO.readNonBlocking((char*)m_ReadBuf+m_actualBufferSize, SCANNER_S300_READ_BUF_SIZE-2-m_actualBufferSize);

	iNumRead = m_actualBufferSize + iNumRead2;
	m_actualBufferSize = m_actualBufferSize + iNumRead2;

	if( iNumRead < m_Param.iDataLength )
	{
		// not enough data in queue --> abort reading
	  //	printf("Not enough data in queue, read data at slower rate!\n");
		return false;
	}
	
	TelegramParser tp;

	// Try to find scan. Searching backwards in the receive queue.
	for(i=iNumRead; i>=0; i--)
	{
		// parse through the telegram until header with correct scan id is found
		if(tp.parseHeader(m_ReadBuf+i, iNumRead-i, m_iScanId))
		{
			tp.readDistRaw(m_ReadBuf+i, m_viScanRaw);
			if(m_viScanRaw.size()>0) {
				// Scan was succesfully read from buffer
				bRet = true;
				m_actualBufferSize = 0;
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
