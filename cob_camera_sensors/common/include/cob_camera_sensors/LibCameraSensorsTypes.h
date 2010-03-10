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
* Description: Defines for camera sensors.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Sept 2008
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
 

/// @file LibCameraSensorTypes.h
/// Defines for camera sensors.
/// @author Jan Fischer
/// @date September 2008.

#ifndef __LIBCAMERASENSORSTYPES_H__
#define __LIBCAMERASENSORSTYPES_H__

#include <iostream>

namespace ipa_CameraSensors {

	/// Enum to encode return codes or return values.
	enum
	{
		/// BEGIN
		/// Do not change to keep consistency with RET_OK and RET_FAILED
		/// definition in ipa_Utils (GlobalDefines.h)
		RET_OK =									0x00000001UL, ///< Everythings OK.
		RET_FAILED =								0x00000002UL, ///< Something went wrong.
		/// END

		RET_FAILED_OPEN_FILE =						0x00000004UL, ///< Could not open file error.
		RET_FAILED_CV_CREATE_CAMERA =				0x00000008UL, ///< Camera object could not be created.
		RET_MISSING_INTRINSIC_DISTORTION_PARAMS =	0x00000010UL, ///< Intrinsic or distortion parameters have not been set.
		RET_FUNCTION_NOT_IMPLEMENTED =              0x00000020UL, ///< The function of the interface is not implemented within the child instance.
		RET_XML_ATTR_NOT_FOUND =					0x00000040UL, ///< An xml attribute has not been found while parsing the xml ini file.
		RET_XML_TAG_NOT_FOUND =						0x00000080UL, ///< An xml tag has not been found while parsing the xml ini file.
		RET_CAMERA_ALREADY_OPEN =					0x00000100UL, ///< Camera has already been opened.
		RET_CAMERA_ALREADY_INITIALIZED =			0x00000200UL, ///< Camera has already been initialized.
		RET_CAMERA_NOT_OPEN =						0x00000400UL, ///< Camera has not been opened.
		RET_CAMERA_NOT_INITIALIZED =				0x00000600UL, ///< Camera has not been initialized.
		RET_INIT_CAMERA_FAILED =					0x00000800UL, ///< Initialization of camera device failed.
		RET_OPEN_CAMERA_FAILED =					0x00001000UL, ///< Opening camera device failed.
		RET_OPEN_CHECK_FAILED =						0x00002000UL, ///< <code>isOpen()</code> check failed.
		RET_INIT_CHECK_FAILED =						0x00004000UL, ///< <code>isInitialized()</code> check failed.
		RET_CLOSE_CAMERA_FAILED =					0x00008000UL, ///< Closing camera device failed.
		RET_GET_COLOR_IMAGE_FAILED =				0x00010000UL, ///< Acquiring a color image failed.
		RET_GET_INTRINSIC_PARAMS_FAILED =			0x00020000UL, ///< Acquiring intrinsic parameters failed.
		RET_SET_INTRINSIC_PARAMS_FAILED =			0x00040000UL, ///< Setting intrinsic parameters failed.
		RET_GET_DISTORTION_COEFFS_FAILED =			0x00060000UL, ///< Acquiring distortion coeffs failed.
		RET_SET_DISTORTION_COEFFS_FAILED =			0x00080000UL, ///< Setting distortion coeffs failed.
		RET_REMOVE_DISTORTION_FAILED =				0x00100000UL, ///< Removeing distortion failed.
		RET_SET_PROPERTY_DEFAULTS_FAILED =			0x00200000UL, ///< Setting property defaults failed.
		RET_SAVE_PARAMS_FAILED =					0x00400000UL, ///< Setting property defaults failed.
		RET_SET_PROPERTY_FAILED =					0x00800000UL, ///< Setting property failed.	
		RET_GET_PROPERTY_FAILED =					0x01000000UL  ///< Getting property failed.	
	};

	/// Enum to encode the different camera types
	enum t_cameraType
	{
		CAM_VIRTUALCOLOR = 0,	///< Virtual color camera
		CAM_VIRTUALRANGE,		///< Virtual range camera
		CAM_IC,					///< Imaging source camera (blue camera)
		CAM_AVTPIKE,			///< AVT Pike camera (red camera)
		CAM_AXIS,			///< Axis 2100 IP camera
		CAM_SR3000,				///< Swissranger 3000/3100/4000 camera
		CAM_PMDCAMCUBE			///< PMD Cam Cube camera
	};

	/// Enum to identify camera device properties
	enum t_cameraPropertyID
	{
		PROP_BRIGHTNESS = 0,		///< An offset is added to the CCD output signal.
		PROP_SHARPNESS,				///< Enhance blurred images.
		PROP_WHITE_BALANCE_U,		///< Varies the degree of red and blue to achieve equal values for
									///< R G and B in case of gray values.
									///< Auto: Balance algorithm affects the video stream continuously.\n
									///< One push: Triggers only one pass of the adaption procedure.        
		PROP_WHITE_BALANCE_V,		///< @see PROP_WHITE_BALANCE_U
		PROP_HUE,					///< Shifts the color values. Relation between colors remains.
		PROP_SATURATION,            ///< Adjusts the color's saturation from monochrome to high color values.
		PROP_GAMMA,					///< Increases/Decreases the middle gray level of an image
		PROP_EXPOSURE_TIME,			///< Specifies the exposure time
		PROP_GAIN,					///< Specifies the amplification of the CCD output signal.   
		PROP_OPTICAL_FILTER,        
		PROP_FRAME_RATE,			///< Specifies the framerate (frames per second)
		PROP_REGISTER,                
		PROP_TIMEOUT,
		PROP_CAMERA_RESOLUTION,		///< Specifies the camera resolution.
		PROP_VIDEO_ALL,				///< Specifies the video format, video mode and color mode (@see t_videoFormat).
		PROP_VIDEO_FORMAT,			///< DCAM specification (FORMAT 0 - FORMAT 7)
		PROP_VIDEO_MODE,			///< DCAM specification (MODE 0 - MODE 7)
		PROP_COLOR_MODE,			///< COLOR_RGB8/16/16S, COLOR_MONO8/16/16S, COLOR_YUV411/422/444, RAW8/16
		PROP_ISO_SPEED,				///< Specifies the designated isochronous speed (100, 200, 400, or 800)
		PROP_FW_OPERATION_MODE,			///< Firewire operation mode (A/B)
		PROP_SHUTTER,				///< Specifies the exposure time

		PROP_AMPLITUDE_THRESHOLD,	///< Specifies the amplitude threshold of the range imaging camera
									///< Enables filtering of noisy pixels. Pixels that do not have an
									///< intensity value (reflected light) higher than the threshold are set to 0.
		PROP_INTEGRATION_TIME,		///< Specifies the exposure time for each of the four internally acquired images 
									///< to determine the actual range data. Values may vary from 0 (0.2 ms) to 255 (51.2 ms)
									///< The higher the integration time, the more accurate the measurement,
									///< but also the lower the framrate.
		PROP_RESOLUTION,			///< Camera resolution
		PROP_MODULATION_FREQUENCY,	///< For range cameras only: Defines the modulation frequency of the simus wave.
									///< Due to phase shift calculations from the reflected and original wave, the distiance is induced
		PROP_ACQUIRE_MODE,			///< Acquire modes for swissranger camera
		PROP_DISTANCE_OFFSET,		///< Offset that is added to the distance values of the range imaging sensor
		PROP_ROI,					///< Region of interest
		PROP_LENS_CALIBRATION,		///< Specifies if to use native lens calibration of manufactorer
		PROP_DMA_BUFFER_SIZE		///< Buffer size of camera module
	};

	/// Enum represents color modes
	enum t_colorMode
	{
		COLOR_YUV444 = 0,
		COLOR_YUV422,
		COLOR_YUV411,
		COLOR_RGB8,
		COLOR_RGB16S,
		COLOR_RGB16,
		COLOR_MONO8,
		COLOR_MONO16S,
		COLOR_MONO16,
		COLOR_RAW8,
		COLOR_RAW16,
		COLOR_DEFAULT ///< Set default value
	};

	/// Enum represents video formats
	enum t_videoFormat
	{
		FORMAT_0 = 0,
		FORMAT_1,
		FORMAT_2,
		FORMAT_7,
		FORMAT_DEFAULT ///< Set default value
	};

	/// Enum represents video modes
	enum t_videoMode
	{
		MODE_0 = 0,
		MODE_1,
		MODE_2,
		MODE_3,
		MODE_4,
		MODE_5,
		MODE_6,
		MODE_7,
		MODE_DEFAULT ///< Set default value
	};

	/// Special values for camera properties
	enum t_specialValues
	{
		VALUE_AUTO = 0,	///< No control of this parameter allowed. Parameter is switched back to manufacturers default value
		VALUE_ONESHOT,	///< Parameter is continuously controlled automatically by the image device itself. A normal value write switches the automatic mode off
		VALUE_OFF,		///< Parameter is adjusted automatically one time and can then be controlled manually again by writing new values
		VALUE_DEFAULT	///< Set default values. This usual means, that nothing is set at all
	};

	/// Enum that specifies the return type of a parameter request
	enum t_cameraPropertyType
	{
		TYPE_CAMERA_RESOLUTION =					0x00000001L, ///< Type camera resolution
		TYPE_VIDEO_FORMAT =							0x00000002L, ///< Type video format
		TYPE_VIDEO_MODE =							0x00000004L, ///< Type video mode
		TYPE_COLOR_MODE =							0x00000008L, ///< Type color mode
		TYPE_CHARACTER =							0x00000010L, ///< Type character
		TYPE_SHORT =								0x00000020L, ///< Type short
		TYPE_INTEGER =								0x00000040L, ///< Type integer
		TYPE_LONG =									0x00000080L, ///< Type long
		TYPE_FLOAT =								0x00000100L, ///< Type float
		TYPE_DOUBLE =								0x00000200L, ///< Type double
		TYPE_UNSIGNED =								0x00000400L, ///< Type unsigned (only in combination with others valid)
		TYPE_STRING =								0x00000800L, ///< Type string
		TYPE_DATA =									0x00001000L, ///< Type data
		TYPE_SPECIAL =								0x00002000L  ///< Type special. @see t_specialValue
	};

	/// Struct to represent a camera resolution
	struct t_cameraResolution
	{
		int xResolution;
		int yResolution;
	};

	/// Struct to represent a general camera property
	struct t_cameraProperty
	{
		t_cameraPropertyID propertyID; ///< ID to identify the defined parameter
		unsigned long propertyType; ///< The type of the property. To enable a OR connection
									///< like (TYPE_UNSIGNED | TYPE_INTEGER), unsigned long
									///< is used as type and not t_cameraPropertyType

		t_cameraResolution cameraResolution; ///< The currently defined camera resolution
		t_cameraResolution* cameraResolutions; ///< Array to store all available camera resolutions
		int count_cameraResolution;

		t_videoFormat videoFormat;
		t_videoMode videoMode;
		t_colorMode colorMode;

		t_specialValues specialValue;

		int integerData;
		short shortData;
		double doubleData;
		float floatData;
		char charData;
		long longData;

		unsigned int u_integerData;
		unsigned short u_shortData;
		unsigned char u_charData;
		unsigned long u_longData;

		std::string stringData;

		void* data;		///< Points to a data array of arbitrary size
		int count_data;	///< Size of the data array

	};

	enum t_CalibrationMethod
	{
		NATIVE = 0,
		MATLAB_NO_Z,
		MATLAB
	};

	enum t_ToFGrayImageType
	{
		INTENSITY = 0,
		AMPLITUDE
	};

	/// Enum to encode the different camera roles
	typedef enum
	{
		MASTER = 0,			///< Initializes and releases the library, emits the trigger signal if possible.
		SLAVE				///< Takes pictures based on the received trigger signal
	}t_cameraRole;
} // namespace ipa_CameraSensors

#endif // __LIBCAMERASENSORSTYPES_H__
