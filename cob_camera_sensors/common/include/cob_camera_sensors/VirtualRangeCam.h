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
* Description: Virtual range camera representation.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Jan 2009
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

/// @file VirtualRangeCam.h
/// Virtual range camera representation.
/// @author Jan Fischer 
/// @date 2009

#ifndef __VIRTUALRANGECAM_H__
#define __VIRTUALRANGECAM_H__

#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>
#include <math.h>
#include <assert.h>
#include <sstream>

#ifdef __COB_ROS__
#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <tinyxml/tinyxml.h>
#else
#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#include <Vision/Extern/TinyXml/tinyxml.h>
#endif

#include <boost/filesystem.hpp>

#ifdef SWIG
%module Sensors3D

%{
	#include "Swissranger.h"
%}
#endif

namespace fs = boost::filesystem;
using namespace ipa_Utils;

namespace ipa_CameraSensors {

static const int SWISSRANGER_COLUMNS = 176;
static const int SWISSRANGER_ROWS = 144;

/// @ingroup VirtualCameraDriver
/// Interface class to virtual range camera like Swissranger 3000/4000.
/// The class offers an interface to a virtual range camera, that is equal to the interface of a real range camera.
/// However, pictures are read from a directory instead of the camera.
class VirtualRangeCam : public AbstractRangeImagingSensor
{
public:

	VirtualRangeCam();
	~VirtualRangeCam();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepOneChannel, char* rangeImage=NULL, char* intensityImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);
	unsigned long AcquireImages(IplImage* rangeImage=NULL, IplImage* intensityImage=NULL,
		IplImage* cartesianImage=NULL, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);
	unsigned long AcquireImages2(IplImage** rangeImage=NULL, IplImage** intensityImage=NULL, 
		IplImage** cartesianImage=NULL, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);

	unsigned long GetCalibratedUV(double x, double y, double z, double& u, double& v);

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

	/// Returns the number of images in the directory
	/// @return The number of images in the directory
	int GetNumberOfImages();

	/// Function specific to virtual camera.
	/// Resets the image directory read from the configuration file.
	/// @param path The camera path
	/// @return Return code
	unsigned long SetPathToImages(std::string path);

private:
	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	unsigned long GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated);
	unsigned long GetCalibratedZNative(int u, int v, IplImage* coordinateImage, float& zCalibrated);
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y);
	unsigned long GetCalibratedXYNative(int u, int v, IplImage* coordinateImage, float& x, float& y);
	
	/// Load general range camera parameters .
	/// @param filename Configuration file-path and file-name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return code
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	bool m_CoeffsInitialized;

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

	std::string m_CameraDataDirectory; ///< Directory where the image data resides
	int m_CameraIndex; ///< Index of the specified camera. Important, when several cameras of the same type are present

	std::vector<std::string> m_AmplitudeImageFileNames;
	std::vector<std::string> m_IntensityImageFileNames;
	std::vector<std::string> m_RangeImageFileNames ;
	std::vector<std::string> m_CoordinateImageFileNames ;

	unsigned int m_ImageCounter; ///< Holds the index of the image that is extracted during the next call of <code>AcquireImages</code>
	int m_ImageWidth;  ///< Image width
	int m_ImageHeight; ///< Image height

	double m_k1, m_k2, m_p1, m_p2; ///< Distortion parameters
};


} // end namespace ipa_CameraSensors
#endif // __VIRTUALRANGECAM_H__


