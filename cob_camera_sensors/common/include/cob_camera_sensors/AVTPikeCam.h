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
* Description: Interface to AVTPikeCam Firewire Camera.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Nov 2008
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
 
/// @file AVTPikeCam.h
/// Interface to AVTPikeCam Firewire Camera.
/// @author Jan Fischer
/// @date Novemeber, 2008


#ifndef __IPA_AVTPIKECAM_H__
#define __IPA_AVTPIKECAM_H__

#ifdef __COB_ROS__
	#include "cob_camera_sensors/AbstractColorCamera.h"
#else
	#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#endif

#include <cstdlib>

#ifdef _WIN32
	#include <fgcamera.h>
	#ifdef __MINGW__
	typedef unsigned char UINT8;
	#endif
#endif

#define UINT8P_CAST(x) reinterpret_cast<UINT8*>(x)

#ifdef __LINUX__
	#include <dc1394/dc1394.h>
	typedef struct
	{
	  unsigned long Low;
	  unsigned long High;
	}UINT32HL;
	#define UINT32 unsigned long
	#define _UINT32HL
#endif

#ifdef SWIG
%module Sensors
%include "Source/Vision/CameraSensors/AbstractColorCamera.h"

%{
	#include "AVTPikeCam.h"
%}
#endif

using namespace std;

namespace ipa_CameraSensors {

/// @ingroup CameraSensorDriver
/// Interface developed for AVT PIKE 145C camera.
/// Interface should also fit to other IEEE 1394 cameras.
class __DLL_LIBCAMERASENSORS__ AVTPikeCam : public AbstractColorCamera
{
	/// @class AVTPikeCamDeleter
	/// We create a static object of deleter in order to
	/// executes in its destructor operation that have to be executed
	/// only once for AVTPike cameras and only when the last object
	/// camera has been destroyed
	class AVTPikeCamDeleter
	{
	public:
		~AVTPikeCamDeleter() 
		{
#ifndef __LINUX__
			FGExitModule();
#endif
		};
	};

	private:
		/// Camera specific parameters
		bool m_operationMode_B; ///< FireWire A (400Mbit/s) or FireWire B (800Mbit/s).
		UINT32HL m_GUID;			///< GUID (worldwide unique identifier) of the IEEE1395 camera
		static bool m_OpenExecuted; ///< Trigger takes care, that AVT library is opend only once
		static AVTPikeCamDeleter m_Deleter; ///< Cleans up stuff that has to done only once independent
											///< of the number of cameras from this type
#ifdef __LINUX__
		dc1394video_frame_t* m_Frame;
		dc1394_t* m_IEEE1394Info;	///< Hold information about IEEE1394 nodes, connected cameras and camera properties
		dc1394camera_list_t* m_IEEE1394Cameras;	///< List holds all available firewire cameras
		dc1394camera_t* m_cam; 		///< Opened camera instance.
		dc1394video_modes_t m_availableVideoModes; ///< Struct holds all available video modes for the opened camera 

		dc1394speed_t m_IsoSpeed;	 		///< ISO speed.
		dc1394framerate_t m_FrameRate;		///< Frame rate.
		dc1394video_mode_t m_VideoMode;		///< Comprise format and mode (i.e. DC1394_VIDEO_MODE_1280x960_RGB8 or DC1394_VIDEO_MODE_FORMAT7_0)
		dc1394color_coding_t m_ColorCoding;	///< Necessary for Format 7 video mode. Here the color coding is not specified through the video mode   
#endif

#ifdef _WIN32
		CFGCamera m_cam;			///< The camera object for AVT FireGrab (part of AVT FirePackage)		
		FGNODEINFO m_nodeInfo[5];	///< Array holds information about all detected firewire nodes
		UINT32 m_NodeCnt;			///< Number of detected IEEE1394 nodes
		FGFRAME m_Frame; ///< The acquired latest frame from the camera
#endif
		/// Parses the XML configuration file, that holds the camera settings
		/// @param filename The file name and path of the configuration file
		/// @param cameraIndex The index of the camera within the configuration file
		///		   i.e. AVT_PIKE_CAM_0 or AVT_PIKE_CAM_1
		/// @return Return value
		unsigned long LoadParameters(const char* filename, int cameraIndex);

		/// Parses the data extracted by <code>LoadParameters</code> and calls the
		/// corresponding <code>SetProperty</code> functions.
		/// @return Return code
		unsigned long SetParameters();

	public:

		/// Constructor
		AVTPikeCam ();
		/// Destructor
		~AVTPikeCam ();

		//*******************************************************************************
		// AbstractColorCamera interface implementation
		//*******************************************************************************

		unsigned long Init(std::string directory, int cameraIndex = 0);

		unsigned long Open();
		unsigned long Close();

		unsigned long GetColorImage(char* colorImageData, bool getLatestFrame);
		unsigned long GetColorImage(cv::Mat* colorImage, bool getLatestFrame);

		unsigned long SaveParameters(const char* filename);		//speichert die Parameter in das File

		/// Sets the camera properties.
		/// The following parameters are admitted:
		///<ol>
		///	<li> PROP_BRIGHTNESS: 0..1023  </li>
		///	<li> PROP_SHUTTER: 0..4095, VALUE_AUTO</li>
		///	<li> PROP_AUTO_EXPOSURE: 50..205</li>
		///	<li> PROP_WHITE_BALANCE_U: 0..568, VALUE_AUTO</li>
		///	<li> PROP_WHITE_BALANCE_V: 0..568, VALUE_AUTO</li>
		///	<li> PROP_HUE: 0..80</li>
		///	<li> PROP_SATURATION: 0..511</li>
		///	<li> PROP_GAMMA: 0..2</li>
		///	<li> PROP_GAIN: 0..680</li>
		///	<li> PROP_FRAME_RATE: 0..60</li>
		///	<li> PROP_FW_OPERATION_MODE: A / B</li>
		///</ol>
		unsigned long SetProperty(t_cameraProperty* cameraProperty);
		unsigned long SetPropertyDefaults();
		unsigned long GetProperty(t_cameraProperty* cameraProperty);

		/// Shows the camera's parameters information.
		/// Shows actual value, max and min value and auto mode for each parameter
		unsigned long PrintCameraInformation();
		unsigned long TestCamera(const char* filename);

		//*******************************************************************************
		// Camera specific functions
		//*******************************************************************************
		
};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_AVTPikeCam();

} // end namespace ipa_CameraSensors

#endif //__IPA_AVTPIKECAM_H__


