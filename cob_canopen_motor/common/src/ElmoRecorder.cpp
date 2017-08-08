/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <math.h>
#include <vector>
#include <stdio.h>
#include <sstream>
#include <cob_canopen_motor/ElmoRecorder.h>
#include <cob_canopen_motor/CanDriveHarmonica.h>

ElmoRecorder::ElmoRecorder(CanDriveHarmonica * pParentHarmonicaDrive) {
	m_pHarmonicaDrive = pParentHarmonicaDrive;

	m_bIsInitialized = false;
	m_iReadoutRecorderTry = 0;
}

ElmoRecorder::~ElmoRecorder() {
}

bool ElmoRecorder::isInitialized(bool initNow) {
	if(initNow) m_bIsInitialized = true;
	return m_bIsInitialized;
}

int ElmoRecorder::configureElmoRecorder(int iRecordingGap, int driveID, int startImmediately){ //iRecordingGap = N indicates that a new sample should be taken once per N time quanta
	m_iDriveID = driveID;

	if(startImmediately >=2 ) startImmediately = 1;

	m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'R', 0, 0);	// Stop Recorder if it's active
	// Record Main speed (index 0, ID1)
	// Active Current (index 9, ID10)
	// Main Position (index 1, ID2)
	// Speed Command (index 15, ID16)
	// RC = 2^(Signal1Index) + 2^(Signal2Index) + ..; e.g.: 2^0 + 2^1 + 2^9 + 2^15 = 33283;
	m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'C', 0, 33283);
	// Set trigger type to immediate
	m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'P', 3, 0);
	// Set Recording Gap
	m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'G', 0, iRecordingGap);
	// Set Recording Length
	// RL = (4096 / Number of Signals)
	m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'L', 0, 1024);

	// Set Time Quantum, Default: RP=0 -> TS * 4; TS is 90us by default
	// m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'P', 0, 0);
	// ----> Total Recording Time = 90us * 4 * RG * RL

	m_pHarmonicaDrive->IntprtSetInt(8, 'R', 'R', 0, startImmediately + 1); //2 launches immediately (8, 'R', 'R', 0, 1) launches at next BG

	m_fRecordingStepSec = 0.000090 * 4 * iRecordingGap;

	return 0;
}

int ElmoRecorder::readoutRecorderTry(int iObjSubIndex) {
	//Request the SR (status register) and begin all the read-out process with this action.
	//SDOData.statusFlag is segData::SDO_SEG_WAITING;

	m_iReadoutRecorderTry = 1;
	m_iCurrentObject = iObjSubIndex;

	m_pHarmonicaDrive->requestStatus();

	return 0;
}

int ElmoRecorder::readoutRecorderTryStatus(int iStatusReg, segData& SDOData) {
	if(m_iReadoutRecorderTry == 0) return 0; //only evaluate the SR, if we are really waiting for it (to save time and not to unintionally start a read-out)

	m_iReadoutRecorderTry = 0;

	//Bits 16-17 of status register contain recorder information
	int iRecorderStatus = (0x30000 & iStatusReg) >> 16;

	if(iRecorderStatus == 0) {
		std::cout << "Recorder " << m_iDriveID << " inactive with no valid data to upload" << std::endl;
		SDOData.statusFlag = segData::SDO_SEG_FREE;
	} else if(iRecorderStatus == 1) {
		std::cout << "Recorder " << m_iDriveID << " waiting for a trigger event" << std::endl;
		SDOData.statusFlag = segData::SDO_SEG_FREE;
	} else if(iRecorderStatus == 2) {
		std::cout << "Recorder " << m_iDriveID << " finished, valid data ready for use" << std::endl;
		readoutRecorder(m_iCurrentObject);
		//already set to SDOData.statusFlag = segData::SDO_SEG_WAITING;
	} else if(iRecorderStatus == 3) {
		std::cout << "Recorder " << m_iDriveID << " is still recording" << std::endl;
		SDOData.statusFlag = segData::SDO_SEG_FREE;
	}

	return 0;
}

int ElmoRecorder::readoutRecorder(int iObjSubIndex){
	//initialize Upload of Recorded Data (object 0x2030)
	int iObjIndex = 0x2030;

	m_pHarmonicaDrive->sendSDOUpload(iObjIndex, iObjSubIndex);
	m_iCurrentObject = iObjSubIndex;

	return 0;
}

int ElmoRecorder::processData(segData& SDOData) {
	int iItemSize = 4;
	int iItemCount = 0;
	unsigned int iNumDataItems = 0;
	bool bCollectFloats = true;
	float fFloatingPointFactor = 0;

	std::vector<float> vfResData[2];

	//see SimplIQ CANopen DS 301 Implementation Guide, object 0x2030

	//HEADER
	//--------------------------------------
	//First 7 Bytes of the data sequence contain header information:
	//Byte 0: First four bits: 4 = Long Int data type, 1 = Half Float data type, 5 = Double Float
	//			Next four bits: Recording frequency in 1 per n * TS => deltaT =  n * 90µsec
	//Byte 2, Byte 3: Number of recorded data points
	//Byte 3 to 6: Floating point factor for data to be multiplied with
	//
	//Byte 7 to Byte (7+ iNumdataItems * 4) contain data

	//B[0]: Time quantum and data type
	switch ((SDOData.data[0] >> 4) ) {
		case 4:
			bCollectFloats = false;
			iItemSize = 4;
			break;
		case 5:
			bCollectFloats = true;
			iItemSize = 4;
			break;
		case 1:
			bCollectFloats = true;
			iItemSize = 2;
			break;
		default:
			bCollectFloats = false;
			iItemSize = 4;
			break;
	}
	std::cout << ">>>>>ElmoRec: HEADER INFOS<<<<<\nData type is: " << (SDOData.data[0] >> 4) << std::endl;

	//std::cout << "fTimeQuantum from Header is " << fTimeQuantum << " m_fRecordingStepSec is " << m_fRecordingStepSec << std::endl;


	//B[1]..[2] //Number of recorded items
	iNumDataItems = (SDOData.data[2] << 8 | SDOData.data[1]);
	//std::cout << "Number of recorded data points: " << iNumDataItems << std::endl;

	//B[3] ... [6] //Floating point factor
	fFloatingPointFactor = convertBinaryToFloat( (SDOData.data[6] << 24) | (SDOData.data[5] << 16) | (SDOData.data[4] << 8) | (SDOData.data[3]) );
	std::cout << "Floating point factor for recorded values is: " << fFloatingPointFactor << std::endl;


	if( ((SDOData.numTotalBytes-7)/iItemSize) != iNumDataItems)
		std::cout << "SDODataSize announced in SDO-Header" << ((SDOData.numTotalBytes-7)/iItemSize) << " differs from NumDataItems by ElmoData-Header" <<  iNumDataItems << std::endl;
	//END HEADER
	//--------------------------------------

	vfResData[0].assign(iNumDataItems, 0.0);
	vfResData[1].assign(iNumDataItems, 0.0);
	iItemCount = 0;

	//extract values from data stream, consider Little Endian conversion for every single object!
	for(unsigned int i=7;i<=SDOData.data.size() - iItemSize; i=i+iItemSize) {
		if(bCollectFloats) {
			if(iItemSize == 4)
				vfResData[1][iItemCount] = fFloatingPointFactor * convertBinaryToFloat( (SDOData.data[i] << 0) | (SDOData.data[i+1] << 8) | (SDOData.data[i+2] << 16) | (SDOData.data[i+3] << 24) );

				//DEBUG
				if(iItemCount == 120)
					std::cout << (unsigned int)( (SDOData.data[i] << 0) | (SDOData.data[i+1] << 8) | (SDOData.data[i+2] << 16) | (SDOData.data[i+3] << 24) ) << std::endl;

			else vfResData[1][iItemCount] = fFloatingPointFactor * convertBinaryToHalfFloat( (SDOData.data[i] << 0) | (SDOData.data[i+1] << 8) | (SDOData.data[i+2] << 16) | (SDOData.data[i+3] << 24) );
			iItemCount ++;
		} else {
			vfResData[1][iItemCount] = fFloatingPointFactor * (float)( (SDOData.data[i] << 0) | (SDOData.data[i+1] << 8) | (SDOData.data[i+2] << 16) | (SDOData.data[i+3] << 24) );
			iItemCount ++;
		}

		vfResData[0][iItemCount] = m_fRecordingStepSec * iItemCount;
	}

	logToFile(m_sLogFilename, vfResData);

	SDOData.statusFlag = segData::SDO_SEG_FREE;
	return 0;
}

int ElmoRecorder::setLogFilename(std::string sLogFileprefix) {
	m_sLogFilename = sLogFileprefix;
	return 0;
}



float ElmoRecorder::convertBinaryToFloat(unsigned int iBinaryRepresentation) {
	//Converting binary-numbers to 32bit float values according to IEEE 754 see http://de.wikipedia.org/wiki/IEEE_754
	int iSign;
	int iExponent;
	unsigned int iMantissa;
	float iNumMantissa = 0.0f;

	if((iBinaryRepresentation & (1 << 31)) == 0) //first bit is sign bit: 0 = +, 1 = -
		iSign = 1;
	else
		iSign = -1;

	iExponent = ((iBinaryRepresentation >> 23) & 0xFF) - 127; //take away Bias(127) for positive and negative exponents

	iMantissa = (iBinaryRepresentation & 0x7FFFFF); //only keep mantissa part of binary number

	iNumMantissa = 1.0f;

	for(int i=1; i<=23; i++) { //calculate decimal places (convert binary mantissa to decimal number
		if((iMantissa & (1 << (23-i))) > 0) {
			iNumMantissa = iNumMantissa + pow(2,(-1)*i);
		}
	}

	return iSign * pow(2,iExponent) * iNumMantissa;
}

float ElmoRecorder::convertBinaryToHalfFloat(unsigned int iBinaryRepresentation) {
	//Converting binary-numbers to 16bit float values according to IEEE 754 see http://de.wikipedia.org/wiki/IEEE_754
	int iSign;
	int iExponent;
	unsigned int iMantissa;
	float iNumMantissa = 0.0f;

	if((iBinaryRepresentation & (1 << 15)) == 0) //first bit is sign bit: 0 = +, 1 = -
		iSign = 1;
	else
		iSign = -1;

	iExponent = ((iBinaryRepresentation >> 10) & 0x1F) - 15; //take away Bias(15) for positive and negative exponents

	iMantissa = (iBinaryRepresentation & 0x3FF); //only keep mantissa part of binary number

	iNumMantissa = 1.0f;

	for(int i=1; i<=10; i++) { //calculate decimal places (convert binary mantissa to decimal number
		if((iMantissa & (1 << (10-i))) > 0) {
			iNumMantissa = iNumMantissa + pow(2,(-1)*i);
		}
	}

	return iSign * pow(2,iExponent) * iNumMantissa;
}

// Function for writing Logfile
int ElmoRecorder::logToFile(std::string filename, std::vector<float> vtValues[]) {
	std::stringstream outputFileName;
	outputFileName << filename << "mot_" << m_iDriveID << "_" << m_iCurrentObject << ".log";

	FILE* pFile;
	//open FileStream
	pFile = fopen(outputFileName.str().c_str(), "w");

	//Check if there was a problem
	if( pFile == NULL )
	{
		std::cout << "Error while writing file: " << outputFileName.str() << " Maybe the selected folder does'nt exist." << std::endl;
	}
	else
	{
		// write all data from vector to file
		for (unsigned int i = 0; i < vtValues[0].size(); i++)
			fprintf(pFile, "%e %e\n", vtValues[0][i], vtValues[1][i]);
		fclose(pFile);
	}

	return true;
}
