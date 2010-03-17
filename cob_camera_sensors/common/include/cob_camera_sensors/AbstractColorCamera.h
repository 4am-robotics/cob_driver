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
* Description: Abstract interface for color cameras.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: May 2008
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

/// @file AbstractColorCamera.h
/// Abstract interface for color cameras.
/// @author Jan Fischer
/// @date May 2008.

#ifndef __ABSTRACTCOLORCAMERA_H__
#define __ABSTRACTCOLORCAMERA_H__

#ifdef __LINUX__
#define __DLL_ABSTRACTCOLORCAMERA_H__ 
#define APIENTRY
#else
	#include <windows.h>
#ifdef __LIBCAMERASENSORS_EXPORT__
	#define __DLL_ABSTRACTCOLORCAMERA_H__ __declspec(dllexport)
	#else
	#define __DLL_ABSTRACTCOLORCAMERA_H__ __declspec(dllimport)
	#endif
#endif

#ifdef __COB_ROS__
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>

#include "tinyxml/tinyxml.h"
#include "cob_vision_utils/CameraSensorTypes.h"
#else
#include <highgui.h>
#include <cv.h>
#include <cxcore.h>

#include "Vision/Extern/TinyXml/tinyxml.h"
#include "Vision/CameraSensors/LibCameraSensorsTypes.h"
#endif

#include <iostream>

namespace ipa_CameraSensors {


/// An interface for common color cameras.
///	All color/mono cameras that are used within the project
/// must derive from this class to guarantee
/// interoperability with the already existing code.
class AbstractColorCamera
{
	public: 

		/// Struct stores the values from the xml camera configuration file
		/// All values may also be set to AUTO or DEFAULT
		struct t_ColorCameraParameters
		{
			ipa_CameraSensors::t_cameraRole m_CameraRole;	///< Master or slave camera
			std::stringstream m_VideoFormat;				///< Format 0,1,2 or 7
			std::stringstream m_VideoMode;					///< Mode 0-7
			std::stringstream m_ColorMode;					///< Mono8/16S/16, YUV411/422, Raw16, RGB8, ...
			std::stringstream m_IsoSpeed;					///< Guaranteed speed of isochronous transfer rate
			std::stringstream m_FrameRate;
			std::stringstream m_Shutter;
			std::stringstream m_WhiteBalanceU;
			std::stringstream m_WhiteBalanceV;
			std::stringstream m_Hue;
			std::stringstream m_Saturation;
			std::stringstream m_Gamma;
			std::stringstream m_ExposureTime;	
			std::stringstream m_Gain;
			std::stringstream m_Brightness;
			std::stringstream m_ImageWidth;
			std::stringstream m_ImageHeight;
		};

		/// Initializes the color camera.
		/// Camera specific constants may be set within the configuration file <I>cameraSensorsIni.xml</I>.
		/// The function has to set the member variable <code>m_initilized</code>.
		/// @param directory Path to the configuration file directory.
		/// @param cameraIndex It is possible to have several cameras of the same type on the system.
		///	       One may us the camera index to apply different configuration files to each of them
		/// @return Return code.
		virtual unsigned long Init(std::string directory, int cameraIndex = 0) =0;
	
		/// Returns true, when <code>Init()</code> has been called on the camera.
		/// @return Camera initialized or not.
		virtual bool isInitialized() {return m_initialized;}

		/// Returns true, when <code>Open()</code> has been called on the camera.
		/// @return Camera opened or not.
		virtual bool isOpen() {return m_open;}

		/// Opens the camera device.
		/// All camera specific parameters for opening the camera should have been set within the <code>Init</code>
		/// function.
		/// @return Return code.
		virtual unsigned long Open() =0;

		/// Close camera device.
		/// @return Return code.
		virtual unsigned long Close() =0; //Save intrinsic params back to File
		
		/// Retrieves image data from the color camera.
		/// @param colorImageData An array to be filled with image data
		/// @param getLatestFrame True, when the latest picture has to be returned. Otherwise, the next picture
		///						  following the last call to <code>getLatestFrame</code> is returned.
		/// @return Return code
		unsigned long GetColorImage(char* colorImageData, bool getLatestFrame=true) {return RET_FAILED;}

		/// Retrieves an OpenCV IplImage from the camera.
		/// IplImage must be created (cvCreateImage) before passing it to GetColorImage.
		/// @param colorImage The image that has been acquired by the camera.
		/// @param getLatestFrame If true, the camera acquires a new frame and returns it.
		///						  Otherwise, the next frame following the last returned frame
		///						  is returned from the internal camera buffer.
		/// @return Return code.
		virtual unsigned long GetColorImage(IplImage* colorImage, bool getLatestFrame=true)=0;

		/// Retrieves an OpenCV IplImage from the camera.
		/// IplImage must be a null pointer before passing it to GetColorImage.
		/// @param colorImage The image that has been acquired by the camera.
		/// @param getLatestFrame If true, the camera acquires a new frame and returns it.
		///						  Otherwise, the next frame following the last returned frame
		///						  is returned from the internal camera buffer.
		/// @return Return code.
		virtual unsigned long GetColorImage2(IplImage** colorImage, bool getLatestFrame=true) {return (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED);}

		/// Returns the camera type.
		/// @return The camera type
		virtual t_cameraType GetCameraType();
	
		/// Function to set properties of the camera sensor.
		/// @param propertyID The ID of the property.
		/// @param cameraProperty The value of the property.
		/// @return Return code.
		virtual unsigned long SetProperty(t_cameraProperty* cameraProperty) =0;

		/// Function to set property defaults of the camera sensor.
		/// @return Return code.
		virtual unsigned long SetPropertyDefaults() =0;

		/// Function to get properties of the camera sensor.
		/// @param propertyID The ID of the property.
		/// @param cameraProperty The value of the property.
		/// @return Return code.
		virtual unsigned long GetProperty(t_cameraProperty* cameraProperty) =0;

		/// Displays camera information on standard output.
		/// Information includes available parameters, color and camera formats.
		/// @return Return code.
		virtual unsigned long PrintCameraInformation() =0;

		/// Saves all parameters on hard disk.
		/// @param filename The filename of the storage.
		/// @return Return code.
		virtual unsigned long SaveParameters(const char* filename)=0;

		/// Unit Test for the camera interface.
		/// Tests each of the single interface functions and displays the output on
		/// standard out.
		/// @param filename Path to the camera initialization xml file.
		/// @return Return code.
		virtual unsigned long TestCamera(const char* filename);

		/// Destructor
		virtual ~AbstractColorCamera();

	protected:
		
		bool m_initialized; ///< True, when the camera has sucessfully been initialized.
		bool m_open;		///< True, when the camera has sucessfully been opend.

		t_ColorCameraParameters m_ColorCameraParameters; ///< Storage for xml configuration file data

		t_cameraType m_CameraType; ///< Camera Type

		unsigned int m_BufferSize; ///< Number of images, the camera buffers internally
	private:

		/// Loads all camera specific parameters from the xml configuration file and saves them in t_ColorCameraParameters.
		/// This function is internally called by Init to load the parameters from the xml configuration file.
		/// @param filename The path to the configuration file.
		/// @return Return code.
		virtual unsigned long LoadParameters(const char* filename, int cameraIndex)=0;

		/// Sets the loaded parameters.
		/// @return Return code.
		virtual unsigned long SetParameters()=0;

};

/// Factory function to create an object of LibObjectDetector
#ifdef __cplusplus
extern "C" {
#endif

__DLL_ABSTRACTCOLORCAMERA_H__ AbstractColorCamera* APIENTRY CreateColorCamera_AVTPikeCam();
__DLL_ABSTRACTCOLORCAMERA_H__ AbstractColorCamera* APIENTRY CreateColorCamera_ICCam();
__DLL_ABSTRACTCOLORCAMERA_H__ AbstractColorCamera* APIENTRY CreateColorCamera_Axis2100Cam();
__DLL_ABSTRACTCOLORCAMERA_H__ AbstractColorCamera* APIENTRY CreateColorCamera_VirtualCam();
__DLL_ABSTRACTCOLORCAMERA_H__ void APIENTRY ReleaseColorCamera(AbstractColorCamera* colorCamera);

#ifdef __cplusplus
}
#endif

} // end namespace
#endif // __ABSTRACTCOLORCAMERA_H_
