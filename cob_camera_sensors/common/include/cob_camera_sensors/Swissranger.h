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
* ROS package name: cob_camera_sensors
* Description: Platform independent interface to MESA Swissranger camera.
* Implementation depends on libusbSR library.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: July 2008
* ToDo:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/
 
/// @file Swissranger.h
/// Platform independent interface to MESA Swissranger camera. Implementation depends on
/// libusbSR library.
/// @author Jan Fischer
/// @date July 2008

#ifndef __IPA_SWISSRANGER_H__
#define __IPA_SWISSRANGER_H__

// Windows
#ifndef __LINUX__
#include <windows.h>
// Windows with MinGW
#ifdef __MINGW__
typedef short __wchar_t;
#endif
#endif

// Linux
#ifdef __LINUX__
typedef unsigned long DWORD;
#endif

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <assert.h>

#include <libMesaSR.h>

#include <sstream>

#ifdef __COB_ROS__
#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <tinyxml/tinyxml.h>
#else
#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <Vision/Extern/TinyXml/tinyxml.h>
#endif

#ifdef SWIG
%module Sensors3D

%{
	#include "Swissranger.h"
%}
#endif


using namespace ipa_Utils;

namespace ipa_CameraSensors {

// former SR31Consts.h entries
#define SAFE_FREE(p)       { if(p) { delete (p); (p)=0; } }
#define SWISSRANGER_COLUMNS 176
#define SWISSRANGER_ROWS 144

/// Callback function to catch annoying debug message from swissranger camera
/// @param srCam Swissranger camera instance
/// @param msg the received message of type CM_XXX
/// @param param is a message specific parameter
/// @param is a message specific pointer
/// @return 
int LibMesaCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data);

/// @ingroup RangeCameraDriver
/// Interface class to SwissRanger camera SR-3000.
/// Platform independent interface to SwissRanger camera SR-3000. Implementation depends on
/// libusbSR library.
class Swissranger : public AbstractRangeImagingSensor
{
public:

	Swissranger();
	~Swissranger();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepOneChannel, char* RangeImage=NULL, char* IntensityImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);
	unsigned long AcquireImages(IplImage* rangeImage=NULL, IplImage* grayImage=NULL,
		IplImage* cartesianImage=NULL, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);
	unsigned long AcquireImages2(IplImage** rangeImage=NULL, IplImage** grayImage=NULL,
		IplImage** cartesianImage=NULL,	bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

private:
	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	unsigned long GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated);
	unsigned long GetCalibratedZSwissranger(int u, int v, int width, float& zCalibrated);
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y);
	unsigned long GetCalibratedXYSwissranger(int u, int v, int width, float& x, float& y);

	/// Load general Swissranger parameters and previously determined calibration parameters.
	/// @param filename Swissranger parameter path and file name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return value 
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	/// Parses the data extracted by <code>LoadParameters</code> and calls the
	/// corresponding <code>SetProperty</code> functions.
	/// @return Return code
	unsigned long SetParameters();

	SRCAM m_SRCam; 			 ///< Handle to USB SR3000 camera
	int m_NumOfImages;		 ///< Number of images the siwssranger returns (i.e. an intensity and a range image)
	ImgEntry* m_DataBuffer;  ///< Image array

	// Stores for cartesian data, when native swissranger calibration is used
	float m_X[SWISSRANGER_COLUMNS * SWISSRANGER_ROWS];
	float m_Y[SWISSRANGER_COLUMNS * SWISSRANGER_ROWS];
	float m_Z[SWISSRANGER_COLUMNS * SWISSRANGER_ROWS];

	bool m_CoeffsInitialized; ///< True, when m_CoeffsAx have been initialized
	bool m_GrayImageAcquireCalled; ///< Is false, when acquiring gray image has not been called, yet

	/// Given a 32 bit swissranger depth value, the real depth value in meteres is given by:
	/// z(u,v)=a0(u,v)+a1(u,v)*d(u,v)+a2(u,v)*d(u,v)^2
	///       +a3(u,v)*d(u,v)^3+a4(u,v)*d(u,v)^4+a5(u,v)*d(u,v)^5
	///       +a6(u,v)*d(u,v)^6;
	DblMatrix m_CoeffsA0; ///< a0 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA1; ///< a1 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA2; ///< a2 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA3; ///< a3 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA4; ///< a4 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA5; ///< a5 z-calibration parameters. One matrix entry corresponds to one pixel
	DblMatrix m_CoeffsA6; ///< a6 z-calibration parameters. One matrix entry corresponds to one pixel
};


} // End namespace ipa_CameraSensors
#endif // __IPA_SWISSRANGER_H__


