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

bool ElmoRecorder::processData(segData& SDOData) {
	int iItemSize = 0;
	int iItemCount= 0;
	unsigned int iNumDataItems = 0;
	bool bCollectFloats;
	
	std::vector<float> vfResData;
	std::vector<int> viResData;
	
	//see SimplIQ CANopen DS 301 Implementation Guide, object 0x2030

	//Header Byte Sequence (Byte0 to Byte 7)
	//--------------------------------------
	//SDOData.data[0] //Variable type for user. Field has no practical significance.
	//SDOData.data[1]
	if(((SDOData.data[0] << 8) | SDOData.data[1]) == 0) {
		bCollectFloats = false;
	} else {
		bCollectFloats = true;
	}
	
	std::cout << "bCollectFloats is " << bCollectFloats << std::endl;
	
	
	//SDOData.data[2] << 8 | SDOData.data[3]; //Data width: number of hex character of single transmitted data item.
	//short integer = two bytes, long integer = four bytes
	iItemSize = (SDOData.data[2] << 8 | SDOData.data[3]) / 2; //2 to get bytes from hex-charaters

	std::cout << "Total Num of data items is " << iItemSize << std::endl;

	//SDOData.data[4] ... [7] //Data length: actual number of transmitted data items.
	iNumDataItems = (SDOData.data[4] << 24) | (SDOData.data[5] << 16) | (SDOData.data[6] << 8) | (SDOData.data[7]);

	std::cout << "Total Num of data items is " << iNumDataItems << std::endl;
	
	if(SDOData.data.size() != iNumDataItems) 
		std::cout << "SDODataSize " << SDOData.data.size() << " differs from Num Data Items! " <<  iNumDataItems << std::endl;
	
	
	if(bCollectFloats) {
		vfResData.assign(iNumDataItems, 0.0);
	} else {
		viResData.assign(iNumDataItems, 0);
	}
	iItemCount = 0;
		
	for(int i=8;i<=SDOData.data.size() - (iItemSize-1); i=i+iItemSize) {
		if(bCollectFloats) {
			vfResData[iItemCount] = convertBinaryToFloat((SDOData.data[i] << 24) | (SDOData.data[i+1] << 16) | (SDOData.data[i+2] << 8) | (SDOData.data[i+3]));
			iItemCount ++;
		} else {
			if(iItemSize==2) viResData[iItemCount] = (SDOData.data[i] << 8) | SDOData.data[i+1];
			else viResData[iItemCount] = (SDOData.data[i] << 24) | (SDOData.data[i+1] << 16) | (SDOData.data[i+2] << 8) | (SDOData.data[i+3]);
			iItemCount ++;
		}
	}
	
	return true;
}

float ElmoRecorder::convertBinaryToFloat(unsigned int iBinaryRepresentation) {
	int iSign;
	unsigned int iExponent;
	unsigned int iMantissa;
	float iNumMantissa = 0;
		
	if((iBinaryRepresentation & (1 << 31)) == 0) 
		iSign = 1;
	else
		iSign = -1;

	iExponent = ((iBinaryRepresentation >> 23) & 0xFF) - 127; //take away Bias for positive and negative exponents
	
	iMantissa = (iBinaryRepresentation & 0x7FFFFF);
	iNumMantissa = 1;
	
	for(int i=1; i<=23; i++) {
		if((iMantissa & (1 << (23-i))) > 0) {
			iNumMantissa = iNumMantissa + 1 * pow(2,-1*i);
		}
	}
	
	return iSign * pow(2,iExponent) * iNumMantissa;
}


