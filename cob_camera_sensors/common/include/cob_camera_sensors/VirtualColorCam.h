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


/// @file VirtualColorCam.h
/// Virtual color camera representation.
/// @author Jan Fischer
/// @date 2009.

#ifndef __IPA_VIRTUALCOLORCAM_H__
#define __IPA_VIRTUALCOLORCAM_H__

#include "StdAfx.h"
#ifdef __LINUX__
	#include "cob_camera_sensors/AbstractColorCamera.h"
#else
	#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#endif

#include <cstdlib>
#include <boost/filesystem.hpp>

namespace ipa_CameraSensors {
/// @ingroup VirtualCameraDriver
/// The class offers an interface to a virtual color camera, that is equivalent
/// to the interface of a real color camera.
/// However, pictures are read from a directory instead of the camera.
class __DLL_LIBCAMERASENSORS__ VirtualColorCam : public AbstractColorCamera
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
		unsigned long GetColorImage(cv::Mat* image, bool getLatestFrame);

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

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_VirtualCam();

} // end namespace ipa_CameraSensors

#endif //__IPA_VIRTUALCOLORCAM_H__


