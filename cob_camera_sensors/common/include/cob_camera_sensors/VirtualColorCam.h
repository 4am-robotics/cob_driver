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
* ROS package name: cob_camera_sensors
* Description: Virtual color camera representation.
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
 

/// @file VirtualColorCam.h
/// Virtual color camera representation.
/// @author Jan Fischer
/// @date 2009.

#ifndef __VIRTUALCOLORCAM_H__
#define __VIRTUALCOLORCAM_H__

#ifdef __COB_ROS__
#include "cob_camera_sensors/AbstractColorCamera.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#endif

#include <vector>
#include <iostream>
#include <cstdlib>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
using namespace std;

#ifdef SWIG
%module Sensors
%include "Source/Vision/CameraSensors/AbstractColorCamera.h"

%{
	#include "VirtualColorCam.h"
%}
#endif

namespace ipa_CameraSensors {
/// @ingroup VirtualCameraDriver
/// The class offers an interface to a virtual color camera, that is equivalent
/// to the interface of a real color camera.
/// However, pictures are read from a directory instead of the camera.
class VirtualColorCam : public AbstractColorCamera
{
	private:
		int m_ImageWidth;
		int m_ImageHeight;

		std::string m_CameraDataDirectory; ///< Directory where the image data resides
		int m_CameraIndex; ///< Index of the specified camera. Important, when several cameras of the same type are present

		std::vector<std::string> m_ColorImageFileNames;

		unsigned int m_ImageCounter; ///< Holds the index of the image that is extracted during the next call of <code>AcquireImages</code>

		/// Parses the XML configuration file, that holds the camera settings
		/// @param filename The file name and path of the configuration file
		/// @param cameraIndex The index of the camera within the configuration file
		///		   i.e. AVT_PIKE_CAM_0 or AVT_PIKE_CAM_1
		/// @return Return value
		unsigned long LoadParameters(const char* filename, int cameraIndex);
		
		unsigned long SetParameters(){return RET_OK;};

	public:

		VirtualColorCam ();
		~VirtualColorCam ();

		//*******************************************************************************
		// AbstractColorCamera interface implementation
		//*******************************************************************************

		unsigned long Init(std::string directory, int cameraIndex = 0);

		unsigned long Open();
		unsigned long Close();

		unsigned long GetColorImage(char* colorImageData, bool getLatestFrame);
		unsigned long GetColorImage(IplImage * Img, bool getLatestFrame);
		unsigned long GetColorImage2(IplImage ** Img, bool getLatestFrame);
		unsigned long SaveParameters(const char* filename);		//speichert die Parameter in das File
		unsigned long SetProperty(t_cameraProperty* cameraProperty);
		unsigned long SetPropertyDefaults();
		unsigned long GetProperty(t_cameraProperty* cameraProperty);
		unsigned long PrintCameraInformation();
		unsigned long TestCamera(const char* filename);

		//*******************************************************************************
		// Camera specific functions
		//*******************************************************************************
		
		/// Returns the number of images in the directory
		/// @return The number of images in the directory
		int GetNumberOfImages();

		/// Function specific to virtual camera.
		/// Resets the image directory read from the configuration file.
		/// @param path The camera path
		/// @return Return code
		unsigned long SetPathToImages(std::string path);

};

} // end namespace ipa_CameraSensors

#endif //__VIRTUALCOLORCAM_H__


