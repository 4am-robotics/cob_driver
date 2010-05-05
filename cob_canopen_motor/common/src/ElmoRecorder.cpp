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
 * ROS stack name: cob3_common
 * ROS package name: canopen_motor
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Köhler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Mar 2010
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

#include <math.h>
#include <vector>
#include <cob_canopen_motor/ElmoRecorder.h>
#include <cob_canopen_motor/CanDriveHarmonica.h>

ElmoRecorder::ElmoRecorder(CanDriveHarmonica * pParentHarmonicaDrive) {
	pHarmonicaDrive = pParentHarmonicaDrive;
	
}

ElmoRecorder::~ElmoRecorder() {
}

int ElmoRecorder::configureElmoRecorder(int iRecordingGap){ //iRecordingGap = N indicates that a new sample should be taken once per N time quanta

	pHarmonicaDrive->IntprtSetInt(8, 'R', 'R', 0, -1);	// Stop Recorder if it's active
	// Record Main speed (index 0, ID1) 
	// Active Current (index 9, ID10)
	// Main Position (index 1, ID2)
	// Speed Command (index 15, ID16)
	// RC = 2^(Signal1Index) + 2^(Signal2Index) + ..; e.g.: 2^0 + 2^1 + 2^9 + 2^15 = 33283;
	pHarmonicaDrive->IntprtSetInt(8, 'R', 'C', 0, 33283);	
	// Set Recording Length
	// RL = (4096 / Number of Signals)
	pHarmonicaDrive->IntprtSetInt(8, 'R', 'L', 0, 1024);
	// Set Time Quantum, Default: RP=0 -> TS * 4; TS is 90us by default
	pHarmonicaDrive->IntprtSetInt(8, 'R', 'P', 0, 0);
	// Set Recording Gap
	pHarmonicaDrive->IntprtSetInt(8, 'R', 'G', 0, iRecordingGap);
	// ----> Total Recording Time = 90us * 4 * RG * RL
	
	// Arm Recorder, by Trigger Event of "BG"-Command (Begin Motion)
	pHarmonicaDrive->IntprtSetInt(8, 'R', 'R', 0, 2); //2 launches immediately (8, 'R', 'R', 0, 1) launches at next BG
	
	m_fRecordingStepSec = 0.000090 * 4 * iRecordingGap;
	
	return 0;
}


int ElmoRecorder::readoutRecorder(int iObjSubIndex){ // iObjSubIndex: shift this bit, according to the specified recording sources in RC
	//initialize Upload of Recorded Data (object 0x2030)
	int iObjIndex = 0x2030;
	pHarmonicaDrive->sendSDOUpload(iObjIndex, iObjSubIndex);
	m_iCurrentObject = iObjSubIndex;
	
	return 0;
}

int ElmoRecorder::processData(segData& SDOData) {
	int iItemSize = 0;
	int iItemCount= 0;
	unsigned int iNumDataItems = 0;
	bool bCollectFloats;
	
	std::vector<float> vfResData[2];
	
	//see SimplIQ CANopen DS 301 Implementation Guide, object 0x2030

	//Header Byte Sequence (Byte0 to Byte 7)
	//--------------------------------------
	//SDOData.data[0] //Variable type for user. Contains info, whether floats or integers are transmitted.
	//SDOData.data[1]
	if(((SDOData.data[0] << 8) | SDOData.data[1]) == 0) {
		bCollectFloats = false;
	} else {
		bCollectFloats = true;
	}
	
	std::cout << ">>>>>HEADER INFOS<<<<<\nbCollectFloats: " << bCollectFloats << std::endl;
	
	
	//SDOData.data[2] << 8 | SDOData.data[3]; //Data width: number of hex character of single transmitted data item.
	//short integer = two bytes, long integer = four bytes
	iItemSize = (SDOData.data[2] << 8 | SDOData.data[3]) / 2; //2 to get bytes from hex-charaters

	std::cout << "Size of one data item: " << iItemSize << std::endl;

	//SDOData.data[4] ... [7] //Data length: actual number of transmitted data items.
	iNumDataItems = (SDOData.data[4] << 24) | (SDOData.data[5] << 16) | (SDOData.data[6] << 8) | (SDOData.data[7]);

	std::cout << "Total Num of data items: " << iNumDataItems << std::endl;
	
	if( ((SDOData.data.size()-8)/iItemSize) != iNumDataItems) 
		std::cout << "SDODataSize " << ((SDOData.data.size()-8)/iItemSize) << " differs from Num Data Items! " <<  iNumDataItems << std::endl;
	
	
	vfResData[0].assign(iNumDataItems, 0.0);
	vfResData[1].assign(iNumDataItems, 0.0);
	iItemCount = 0;
	
	//extract values from data stream, consider Little Endian conversion for every single object!
	for(int i=8;i<=SDOData.data.size() - (iItemSize-1); i=i+iItemSize) {
		if(bCollectFloats) {
			vfResData[iItemCount][1] = convertBinaryToFloat( (SDOData.data[i] << 0) | (SDOData.data[i+1] << 8) | (SDOData.data[i+2] << 16) | (SDOData.data[i+3] << 24) );
			iItemCount ++;
		} else {
			if(iItemSize==2) vfResData[iItemCount][1] = (float)((SDOData.data[i] << 0) | (SDOData.data[i+1] << 8)); //collect 2-byte integers (short int)
			else vfResData[iItemCount][1] = (float)((SDOData.data[i] << 0) | (SDOData.data[i+1] << 8) | (SDOData.data[i+2] << 16) | (SDOData.data[i+3] << 24)); //collect 4-byte integers (long int)
			iItemCount ++;
		}
		
		vfResData[iItemCount][0] = m_fRecordingStepSec * i;
	}
	
	logToFile(sLogFilename, vfResData);

	return 0;
}

float ElmoRecorder::convertBinaryToFloat(unsigned int iBinaryRepresentation) {
	//Converting binary-numbers to 32bit float values according to IEEE 754 see http://de.wikipedia.org/wiki/IEEE_754
	int iSign;
	unsigned int iExponent;
	unsigned int iMantissa;
	float iNumMantissa = 0;

	if((iBinaryRepresentation & (1 << 31)) == 0) //first bit is sign bit: 0 = +, 1 = -
		iSign = 1;
	else
		iSign = -1;

	iExponent = ((iBinaryRepresentation >> 23) & 0xFF) - 127; //take away Bias(127) for positive and negative exponents
	
	iMantissa = (iBinaryRepresentation & 0x7FFFFF); //only keep mantissa part of binary number
	iNumMantissa = 1;
	
	for(int i=1; i<=23; i++) { //calculate decimal places (convert binary mantissa to decimal number
		if((iMantissa & (1 << (23-i))) > 0) {
			iNumMantissa = iNumMantissa + 1 * pow(2,-1*i);
		}
	}
	
	return iSign * pow(2,iExponent) * iNumMantissa;
}


// Function for writing Logfile
int ElmoRecorder::logToFile(std::string filename, std::vector<float> vtValues[]) {
    filename = filename + "_" + (char)m_iCurrentObject + ".log";

	FILE* pFile;
	//open FileStream
	pFile = fopen(filename.c_str(), "w");
	
	//Check if there was a problem
	if( pFile == NULL ) 
	{	
		std::cout << "Error while writing file: " << filename << " Maybe the selected folder does'nt exist." << std::endl;
	} 
	else 
	{
		// write all data from vector to file
		for (unsigned int i = 0; i < vtValues[0].size(); i++)
			fprintf(pFile, "%e %e\n", vtValues[i][0], vtValues[i][1]);
	}
	fclose(pFile);
	return true;
}
