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
 * Description: This class provides functions to read out and log recorded data by the built in Elmo Recorder via CAN-protocol.
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

#ifndef _ElmoRecorder_H
#define _ElmoRecorder_H

#include <string>
#include <cob_canopen_motor/SDOSegmented.h>

class CanDriveHarmonica;

class ElmoRecorder {
	public:
		
		ElmoRecorder(CanDriveHarmonica * pParentHarmonicaDrive);

		~ElmoRecorder();
		
		int processData(segData& SDOData);
		
		/** 
		* Configures the Elmo Recorder (internal) to log internal data with high frequency
		* (This can be used for identification of the drive chain)
		* @param iRecordingGap iRecordingGap = N indicates that a new sample should be taken once per N time quanta (= 4 * 90 usec)
		*/
		int configureElmoRecorder(int iRecordingGap, int driveID, int startImmediately = 1);
		
		bool isInitialized(bool initNow);
		
		int readoutRecorder(int iObjSubIndex);
		
		int readoutRecorderTry(int iObjSubIndex);

		int readoutRecorderTryStatus(int iStatusReg);
		
		int logToFile(std::string filename, std::vector<float> vtValues[]);
		
		//Attributes
		std::string sLogFilename;
		
	private:
		int m_iCurrentObject;
		
		float m_fRecordingStepSec;
		
		int m_iReadoutRecorderTry;
	
		CanDriveHarmonica* pHarmonicaDrive;
		
		int m_iDriveID;
		
		bool m_bIsInitialized;
		
		float convertBinaryToFloat(unsigned int binaryRepresentation);
};

#endif
