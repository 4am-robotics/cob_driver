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
* Description:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Mai 2008
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
 
#ifdef __COB_ROS__
#include "cob_camera_sensors/Swissranger.h"
#else
#include "Swissranger.h"
#endif

using namespace ipa_CameraSensors;

/// Entry point function for enduser
#ifdef __cplusplus
extern "C" {
#endif
__DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ AbstractRangeImagingSensor* APIENTRY CreateRangeImagingSensor_Swissranger()
{
	return (new Swissranger());
}
#ifdef __cplusplus
}
#endif


Swissranger::Swissranger()
{
	m_initialized = false;
	m_open = false;

	m_intrinsicMatrix = 0;
	m_distortionParameters = 0;

	m_SRCam = 0;
	m_DataBuffer = 0;
	m_BufferSize = 1;
	m_Fake = false;

	m_CoeffsInitialized = false;
}


Swissranger::~Swissranger()
{
	if (isOpen())
	{
		Close();
	}
}

int ipa_CameraSensors::LibMesaCallback(SRCAM srCam, unsigned int msg, unsigned int param, void* data)
{
	switch(msg)
	{
		case CM_MSG_DISPLAY: /// Redirects all output to console
		{
			if (param==MC_ETH)
			{
				/// Do nothing
				return 0;
			}
			else
			{
				return SR_GetDefaultCallback()(0,msg,param,data);
			}
			break;
		}
		default:
		{
			/// Default handling
			return SR_GetDefaultCallback()(0,msg,param,data);
		}
	}
	return 0;
}


unsigned long Swissranger::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_SR3000;

	/// Load SR parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "INFO - Swissranger::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);	
	}
	
	m_CoeffsInitialized = true;

	/// Set callback function, to catch annoying ethernet messages
	SR_SetCallback(LibMesaCallback);

	if (m_CalibrationMethod == MATLAB)
	{
		/// Load z-calibration files
		if(m_CoeffsA0.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA0.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA0.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}

		/// Load z-calibration files
		if(m_CoeffsA1.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA1.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA1.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}

		/// Load z-calibration files
		if(m_CoeffsA2.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA2.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA2.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}

		/// Load z-calibration files
		if(m_CoeffsA3.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA3.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA3.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}

		/// Load z-calibration files
		if(m_CoeffsA4.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA4.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA4.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}

		/// Load z-calibration files
		if(m_CoeffsA5.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA5.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA5.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}/// Load z-calibration files

		if(m_CoeffsA6.Load(directory + "MatlabCalibrationData/SR/ZCoeffsA6.txt") & RET_FAILED)
		{
			std::cerr << "ERROR - Swissranger::Init:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA6.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
	}
	
	// set init flag
	m_initialized = true;
	m_GrayImageAcquireCalled = false;

	return RET_OK;
}


unsigned long Swissranger::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	std::string sInterface = "";
	m_RangeCameraParameters.m_Interface.clear(); /// Clear flags
	m_RangeCameraParameters.m_Interface.seekg(0); /// Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_Interface >> sInterface;

	if (sInterface == "USB")
	{
		if(SR_OpenUSB(&m_SRCam, 0)<=0)
		{
			std::cerr << "ERROR - Swissranger::Open():" << std::endl;
			std::cerr << "\t ... Could not open swissranger camera on USB port" << std::endl;
			std::cerr << "\t ... Unplug and Replugin camera power cable.\n";
			return RET_FAILED;
		}
	}
	else if (sInterface == "ETHERNET")
	{
		std::string sIP = "";
		m_RangeCameraParameters.m_IP.clear(); /// Clear flags
		m_RangeCameraParameters.m_IP.seekg(0); /// Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_IP >> sIP;
		if(SR_OpenETH(&m_SRCam, sIP.c_str())<=0)
		{
			std::cerr << "ERROR - Swissranger::Open():" << std::endl;
			std::cerr << "\t ... Could not open swissranger camera on ETHERNET port" << std::endl;
			std::cerr << "\t ... with ip '" << sIP << "'." << std::endl;
			std::cerr << "\t ... Unplug and Replugin camera power cable to fix problem\n";
			return RET_FAILED;
		}
	}
	else
	{
		std::cerr << "ERROR - Swissranger::Open():" << std::endl;
		std::cerr << "\t ... Unknown interface type '" << sInterface << "'" << std::endl;
		return RET_FAILED;
	}

	//char DevStr[1024];
	//SR_GetDeviceString(m_SRCam, DevStr, 1024);
	//std::cout << "Swissranger::Open(): INFO" << std::endl;
	//std::cout << "\t ... " << DevStr << std::endl;

	if (SetParameters() & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t ... Could not set parameters" << std::endl;
		return RET_FAILED;
	}

	std::cout << "**************************************************" << std::endl;
	std::cout << "Swissranger::Open: Swissranger camera device OPEN" << std::endl;
	std::cout << "**************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long Swissranger::Close()
{
	if (!isOpen())
	{
		return (RET_OK);
	}

	if(SR_Close(m_SRCam)<0)
	{
		std::cout << "ERROR - Swissranger::Close():" << std::endl;
		std::cerr << "\t ... Could not close swissranger SR3000 camera." << std::endl;
		return RET_FAILED;
	}
	m_SRCam = 0;
	
	m_open = false;
	return RET_OK;

}


unsigned long Swissranger::SetProperty(t_cameraProperty* cameraProperty) 
{
	if (!m_SRCam)
	{
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int err = 0;
	switch (cameraProperty->propertyID)
	{
		case PROP_AMPLITUDE_THRESHOLD:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					unsigned short val = 0;
					err =SR_SetAmplitudeThreshold(m_SRCam, val);
					if(err<0)
					{
						std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set amplitude threshold to AUTO mode" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_SHORT | ipa_CameraSensors::TYPE_UNSIGNED))
			{
				err =SR_SetAmplitudeThreshold(m_SRCam, cameraProperty->u_shortData);
				if(err<0)
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set amplitude threshold to AUTO mode" << std::endl;
					return RET_FAILED;
				}
			}
			else
			{
				std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_SHORT|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		case PROP_INTEGRATION_TIME:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					err = SR_SetAutoExposure(m_SRCam, 1, 150, 5, 40);
					if(err<0)
					{
						std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set integration time to AUTO mode" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_CHARACTER | ipa_CameraSensors::TYPE_UNSIGNED))
			{
				err = SR_SetAutoExposure(m_SRCam, 0xff, 150, 5, 70);
				if(err<0)
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not turn off auto exposure" << std::endl;
					return RET_FAILED;
				}
				err = SR_SetIntegrationTime(m_SRCam, cameraProperty->u_charData);
				if(err<0)
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set amplitude threshold to '" << cameraProperty->u_charData << "'" << std::endl;
					return RET_FAILED;
				}
			}
			else
			{
				std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		case PROP_MODULATION_FREQUENCY:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_LAST);
					if(err<0)
					{
						std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set modulation frequency to AUTO mode" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_STRING))
			{
				/// MF_40MHz, SR3k: maximal range 3.75m
                /// MF_30MHz, SR3k, SR4k: maximal range 5m
                /// MF_21MHz, SR3k: maximal range 7.14m
                /// MF_20MHz, SR3k: maximal range 7.5m
                /// MF_19MHz, SR3k: maximal range 7.89m
                /// MF_60MHz, SR4k: maximal range 2.5m 
                /// MF_15MHz, SR4k: maximal range 10m
                /// MF_10MHz, SR4k: maximal range 15m
                /// MF_29MHz, SR4k: maximal range 5.17m
                /// MF_31MHz
				if (cameraProperty->stringData == "MF_40MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_40MHz);
				}
				else if (cameraProperty->stringData == "MF_30MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_30MHz);
				}
				else if (cameraProperty->stringData == "MF_21MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_21MHz);
				}
				else if (cameraProperty->stringData == "MF_20MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_20MHz);
				}
				else if (cameraProperty->stringData == "MF_19MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_19MHz);
				}
				else if (cameraProperty->stringData == "MF_60MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_60MHz);
				}
				else if (cameraProperty->stringData == "MF_15MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_15MHz);
				}
				else if (cameraProperty->stringData == "MF_10MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_10MHz);
				}
				else if (cameraProperty->stringData == "MF_29MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_29MHz);
				}
				else if (cameraProperty->stringData == "MF_31MHz")
				{
					err = SR_SetModulationFrequency(m_SRCam, MF_31MHz);
				}
				else
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Modulation frequency " << cameraProperty->stringData << " unknown" << std::endl;
				}
				
				if(err<0)
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set modulation frequency " << cameraProperty->stringData << std::endl;
					return RET_FAILED;
				}
				
			}
			else
			{
				std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		case PROP_ACQUIRE_MODE:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_INTEGER)
			{
				err = SR_SetMode(m_SRCam, cameraProperty->integerData);
				if(err<0)
				{
					std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set acquire mode" << std::endl;
					return RET_FAILED;
				}
			}
			else
			{
				std::cerr << "ERROR - Swissranger::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_INTEGER' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		default: 				
			std::cout << "Swissranger::SetProperty: Property " << cameraProperty->propertyID << " unspecified.\n";
			return RET_FAILED;
			break;
	}

	return RET_OK;
}


unsigned long Swissranger::SetPropertyDefaults() 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}


unsigned long Swissranger::GetProperty(t_cameraProperty* cameraProperty) 
{
	switch (cameraProperty->propertyID)
	{
		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;
		case PROP_AMPLITUDE_THRESHOLD:
			if (isOpen())
			{
				cameraProperty->u_shortData = SR_GetAmplitudeThreshold(m_SRCam);
				cameraProperty->propertyType = (TYPE_UNSIGNED | TYPE_SHORT);
			}
			else
			{
				return (RET_FAILED | RET_CAMERA_NOT_OPEN);
			}
			break;

		case PROP_INTEGRATION_TIME:	
			if (isOpen())
			{
				cameraProperty->u_charData = SR_GetIntegrationTime(m_SRCam);
				cameraProperty->propertyType = (TYPE_UNSIGNED | TYPE_CHARACTER);	
			}
			else
			{
				return (RET_FAILED | RET_CAMERA_NOT_OPEN);
			}
			break;
			
		case PROP_ACQUIRE_MODE:	
			if (isOpen())
			{
				cameraProperty->integerData = SR_GetMode(m_SRCam);
				cameraProperty->propertyType = TYPE_INTEGER;	
			}
			else
			{
				return (RET_FAILED | RET_CAMERA_NOT_OPEN);
			}
			break;

		case PROP_CAMERA_RESOLUTION:
			if (isOpen())
			{
				cameraProperty->cameraResolution.xResolution = (int)SR_GetCols(m_SRCam);
				cameraProperty->cameraResolution.yResolution = (int)SR_GetRows(m_SRCam);
				cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			}
			else
			{
				cameraProperty->cameraResolution.xResolution = 176;
				cameraProperty->cameraResolution.yResolution = 144;
				cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			}
			break;

		default: 				
			std::cout << "ERROR - Swissranger::GetProperty:" << std::endl;
			std::cout << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return RET_FAILED;
			break;

	}

	return RET_OK;
}


/// Wrapper for IplImage retrival from AcquireImage
/// Images have to be initialized prior to calling this function
unsigned long Swissranger::AcquireImages(IplImage* rangeImage, IplImage* grayImage, IplImage* cartesianImage, 
										 bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* grayImageData = 0;
	char* cartesianImageData = 0;
	int widthStepOneChannel = -1;

	int width = -1;
	int height = -1;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	GetProperty(&cameraProperty);
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	if(rangeImage)
	{
		if(rangeImage->depth == IPL_DEPTH_32F &&
			rangeImage->nChannels == 1 &&
			rangeImage->width == width &&
			rangeImage->height == height)
		{
			rangeImageData = rangeImage->imageData;
			widthStepOneChannel = rangeImage->widthStep;
		}
		else
		{
			std::cerr << "ERROR - Swissranger::AcquireImages:" << std::endl;
			std::cerr << "\t ... Could not acquire range image. Wrong image attributes." << std::endl;
			return RET_FAILED;
		}
	}

	if(grayImage)
	{
		if(grayImage->depth == IPL_DEPTH_32F &&
			grayImage->nChannels == 1 &&
			grayImage->width == width &&
			grayImage->height == height)
		{
			grayImageData = grayImage->imageData;
			widthStepOneChannel = grayImage->widthStep;
		}
		else
		{
			std::cerr << "ERROR - Swissranger::AcquireImages:" << std::endl;
			std::cerr << "\t ... Could not acquire intensity image. Wrong image attributes." << std::endl;
			return RET_FAILED;
		}
	}	

	if(cartesianImage)
	{
		if(cartesianImage->depth == IPL_DEPTH_32F &&
			cartesianImage->nChannels == 3 &&
			cartesianImage->width == width &&
			cartesianImage->height == height)
		{
			cartesianImageData = cartesianImage->imageData;
			widthStepOneChannel = cartesianImage->widthStep/3;
		}
		else
		{
			std::cout << "ERROR - Swissranger::AcquireImages:" << std::endl;
			std::cerr << "\t ... Could not acquire cartesian image. Wrong image attributes." << std::endl;
			return RET_FAILED;
		}
	}

	if (widthStepOneChannel == 0)
	{
		return RET_OK;
	}

	return AcquireImages(widthStepOneChannel, rangeImageData, grayImageData, cartesianImageData, getLatestFrame, undistort, grayImageType);
	
}

/// Wrapper for IplImage retrival from AcquireImage
unsigned long Swissranger::AcquireImages2(IplImage** rangeImage, IplImage** grayImage, IplImage** cartesianImage,
										  bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* grayImageData = 0;
	char* cartesianImageData = 0;
	int widthStepOneChannel = -1;

	int width = -1;
	int height = -1;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	GetProperty(&cameraProperty);
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	if (rangeImage)
	{
		*rangeImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
		rangeImageData = (*rangeImage)->imageData;
		widthStepOneChannel = (*rangeImage)->widthStep;
	} 

	if(grayImage)
	{
		*grayImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
		grayImageData = (*grayImage)->imageData;
		widthStepOneChannel = (*grayImage)->widthStep;
	}

	if(cartesianImage)
	{
		*cartesianImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
		cartesianImageData = (*cartesianImage)->imageData;
		widthStepOneChannel = ((*cartesianImage)->widthStep)/3;
	}

	if (widthStepOneChannel == -1)
	{
		return RET_OK;
	}
	
	return AcquireImages(widthStepOneChannel, rangeImageData, grayImageData, cartesianImageData, getLatestFrame, undistort, grayImageType);
}

// Enables faster image retrival than AcquireImage
unsigned long Swissranger::AcquireImages(int widthStepOneChannel, char* rangeImageData, char* grayImageData, char* cartesianImageData,
										 bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
///***********************************************************************
/// Get data from camera
///***********************************************************************
	if (!m_open)
	{
		std::cerr << "ERROR - Swissranger::AcquireImages:" << std::endl;
		std::cerr << "t ... Camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	//unsigned int c = SR_GetIntegrationTime(m_SRCam);
	//unsigned short a = SR_GetAmplitudeThreshold(m_SRCam);
	//std::cout << "\t ... Integration time is '" << c << "'" << std::endl;
	//std::cout << "\t ... Amplitude threshold is '" << a << "'" << std::endl;

	int width = -1;
	int height = -1;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	GetProperty(&cameraProperty);
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	unsigned int bytesRead	= 0;
	bytesRead = SR_Acquire(m_SRCam);
	if(bytesRead <= 0)
	{
		std::cerr << "ERROR - Swissranger::AcquireImages:" << std::endl;
		std::cerr << "\t ... Could not acquire image!" << std::endl;
		return RET_FAILED;
	}

	if (getLatestFrame == true)
	{
		bytesRead = SR_Acquire(m_SRCam);
		if(bytesRead <= 0)
		{
			std::cerr << "ERROR - Swissranger::AcquireImages:" << std::endl;
			std::cerr << "\t ... Could not acquire image!" << std::endl;
			return RET_FAILED;
		}
	}
	WORD* pixels =(WORD*) SR_GetImage(m_SRCam, 0);
///***********************************************************************
/// Range image (distorted or undistorted)
///***********************************************************************
	if (rangeImageData)
	{
		int widthStepRangeImage = widthStepOneChannel;
		float rangeValue = -1;
		
		// put data in corresponding IPLImage structures
		for(unsigned int row=0; row<(unsigned int)height; row++)
		{
			for (unsigned int col=0; col<(unsigned int)width; col++)
			{
				rangeValue = (float)(pixels[width*row + col]);
				((float*) (rangeImageData + row*widthStepRangeImage))[col] = rangeValue;
			}	
		}
		
		if (undistort)
		{
			CvMat* undistortedData = cvCreateMat(height, width, CV_32FC1 );
			undistortedData->data.fl = (float*) rangeImageData;

			CvMat* distortedData = cvCloneMat(undistortedData);
 
			RemoveDistortion(distortedData, undistortedData);

			cvReleaseMat(&distortedData);
		}

	} // End if (rangeImage)

///***********************************************************************
/// Intensity/Amplitude image
/// ATTENTION: SR provides only amplitude information
///***********************************************************************
	if(grayImageData)
	{
		if (grayImageType == ipa_CameraSensors::INTENSITY &&
			m_GrayImageAcquireCalled == false)
		{
			std::cout << "WARNING - Swissranger::AcquireImages:" << std::endl;
			std::cout << "\t ... Intensity image for swissranger not available" << std::endl;
			std::cout << "\t ... falling back to amplitude image" << std::endl;
			m_GrayImageAcquireCalled = true;
		}

		int widthStepGrayImage = widthStepOneChannel;
		float grayValue = 0;
		int imageSize = width*height;

		for(unsigned int row=0; row<(unsigned int)height-1; row++)
		{
			for (unsigned int col=0; col<(unsigned int)width-1; col++)
			{
				grayValue = (float)(pixels[imageSize+row*width+col]);
				((float*) (grayImageData + row*widthStepGrayImage))[col] = grayValue;
				//cvSetReal2D(*grayImage, row, col, grayValue);
			}	
		}
		
		if (undistort)
		{
			CvMat* undistortedData = cvCreateMat( height, width, CV_32FC1 );
			undistortedData->data.fl = (float*) grayImageData;

			CvMat* distortedData = cvCloneMat(undistortedData);
 
			RemoveDistortion(distortedData, undistortedData);

			cvReleaseMat(&distortedData);
		}

	}

///***********************************************************************
/// Cartesian image (always undistorted)
///***********************************************************************
	if(cartesianImageData)
	{
		int widthStepCartesianImage = widthStepOneChannel*3;
		float x = -1;
		float y = -1;
		float zRaw = -1;
		float zCalibrated = -1;
		float* ptr = 0;

		if(m_CalibrationMethod==MATLAB)
		{
			if (m_CoeffsInitialized)
			{
				/// Calculate calibrated z values (in meter) based on 6 degree polynomial approximation
				CvMat* distortedData = cvCreateMat( height, width, CV_32FC1 );
				for(unsigned int row=0; row<(unsigned int)height; row++)
				{
					for (unsigned int col=0; col<(unsigned int)width; col++)
					{
						zRaw = (float)(pixels[width*row + col]);
						GetCalibratedZMatlab(col, row, zRaw, zCalibrated);
						((float*) (distortedData->data.ptr + row*widthStepOneChannel))[col] = zCalibrated;
					}	
				}
				/*IplImage dummy;
				IplImage *z = cvGetImage(distortedData, &dummy);
				IplImage *image = cvCreateImage(cvGetSize(z), IPL_DEPTH_8U, 3);
				ipa_Utils::ConvertToShowImage(z, image, 1);
				cvNamedWindow("Z");
				cvShowImage("Z", image);
				cvWaitKey();
				*/

				/// Undistort
				CvMat* undistortedData = cvCloneMat(distortedData);
	 			RemoveDistortion(distortedData, undistortedData);
				cvReleaseMat(&distortedData);
				
				/*IplImage dummy;
				IplImage* z = cvGetImage(undistortedData, &dummy);
				IplImage* image = cvCreateImage(cvGetSize(z), IPL_DEPTH_8U, 3);
				ipa_utils::ConvertToShowImage(z, image, 1);
				cvNamedWindow("Z");
				cvShowImage("Z", image);
				cvWaitKey();*/

				/// Calculate X and Y based on instrinsic rotation and translation
				for(unsigned int row=0; row<(unsigned int)height; row++)
				{
					for (unsigned int col=0; col<(unsigned int)width; col++)
					{
						zCalibrated = undistortedData->data.fl[width*row + col];

						GetCalibratedXYMatlab(col, row, zCalibrated, x, y);

						ptr = &((float*) (cartesianImageData + row*widthStepCartesianImage))[col*3];
						ptr[0] = x;
						ptr[1] = y;
						ptr[2] = zCalibrated;
						//CvScalar Val = cvScalar(x, y, z);
						//cvSet2D(*cartesianImage, row, col, Val);
					}
				}
				cvReleaseMat(&undistortedData);
			}
			else
			{
				std::cerr << "ERROR - Swissranger::AcquireImages: \n";
				std::cerr << "\t ... At least one of m_CoeffsA0 ... m_CoeffsA6 not initialized.\n";
				return RET_FAILED;
			}

		}
		else if(m_CalibrationMethod==MATLAB_NO_Z)
		{
			SR_CoordTrfFlt(m_SRCam, m_X, m_Y, m_Z, sizeof(float), sizeof(float), sizeof(float));
			/// Calculate calibrated z values (in meter) based on 6 degree polynomial approximation
			CvMat* distortedData = cvCreateMat( height, width, CV_32FC1 );
			for(unsigned int row=0; row<(unsigned int)height; row++)
			{
				for (unsigned int col=0; col<(unsigned int)width; col++)
				{
					GetCalibratedZSwissranger(col, row, width, zCalibrated);
					((float*) (distortedData->data.ptr + row*widthStepOneChannel))[col] = zCalibrated;
				}	
			}

			/// Undistort
			CvMat* undistortedData = cvCloneMat(distortedData);
 			RemoveDistortion(distortedData, undistortedData);
			cvReleaseMat(&distortedData);

			/// Calculate X and Y based on instrinsic rotation and translation
			for(unsigned int row=0; row<(unsigned int)height; row++)
			{
				for (unsigned int col=0; col<(unsigned int)width; col++)
				{
					zCalibrated = undistortedData->data.fl[width*row + col];

					GetCalibratedXYMatlab(col, row, zCalibrated, x, y);

					ptr = &((float*) (cartesianImageData + row*widthStepCartesianImage))[col*3];
					ptr[0] = x;
					ptr[1] = y;
					ptr[2] = zCalibrated;
					//CvScalar Val = cvScalar(x, y, z);
					//cvSet2D(*cartesianImage, row, col, Val);
				}
			}
			cvReleaseMat(&undistortedData);
		}
		else if(m_CalibrationMethod==NATIVE)
		{
			SR_CoordTrfFlt(m_SRCam, m_X, m_Y, m_Z, sizeof(float), sizeof(float), sizeof(float));
					
			for(unsigned int row=0; row<(unsigned int)height; row++)
			{
				for (unsigned int col=0; col<(unsigned int)width; col++)
				{
					GetCalibratedZSwissranger(col, row, width, zCalibrated);
					GetCalibratedXYSwissranger(col, row, width, x, y);

					//CvScalar Val = cvScalar(x, y, z);
					//cvSet2D(*cartesianImage, row, col, Val);
					ptr = &((float*) (cartesianImageData + row*widthStepCartesianImage))[col*3];
					ptr[0] = x;
					ptr[1] = y;
					ptr[2] = zCalibrated;
				}
			}
		}
		else
		{
			std::cerr << "ERROR - Swissranger::AcquireImages:" << std::endl;
			std::cerr << "\t ... Calibration method unknown.\n";
			return RET_FAILED;
		}
	}
	return RET_OK;
}

unsigned long Swissranger::SaveParameters(const char* filename) 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long Swissranger::GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated)
{
	if (!m_Fake)
	{
		double c[7] = {m_CoeffsA0[v][u], m_CoeffsA1[v][u], m_CoeffsA2[v][u], 
			m_CoeffsA3[v][u], m_CoeffsA4[v][u], m_CoeffsA5[v][u], m_CoeffsA6[v][u]};
		double y = 0;
		ipa_Utils::EvaluatePolynomial((double) zRaw, 6, &c[0], &y);
		zCalibrated = (float) y;
	}
	else
	{
		zCalibrated = 100;
	}
	return RET_OK;
}

/// Return value is in m
unsigned long Swissranger::GetCalibratedZSwissranger(int u, int v, int width, float& zCalibrated)
{
	if (!m_Fake)
	{
		zCalibrated = (float) m_Z[v*width + u];
	}
	else
	{
		zCalibrated = 100;
	}
	return RET_OK;
}

/// u and v are assumed to be distorted coordinates
unsigned long Swissranger::GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y)
{
	/// Conversion form m to mm
	z *= 1000;

	/// Use intrinsic camera parameters
	double fx, fy, cx, cy;
	fx = cvmGet(m_intrinsicMatrix,0,0);
	fy = cvmGet(m_intrinsicMatrix,1,1);

	cx = cvmGet(m_intrinsicMatrix,0,2);
	cy = cvmGet(m_intrinsicMatrix,1,2);

	/// Fundamental equation: u = (fx*x)/z + cx
	if (fx == 0)
	{
		std::cerr << "ERROR - Swissranger::GetCalibratedXYZ:" << std::endl;
		std::cerr << "\t ... fx is 0.\n";
		return RET_FAILED;
	}
	x = (float) (z*(u-cx)/fx) ; 
	
	/// Fundamental equation: v = (fy*y)/z + cy
	if (fy == 0)
	{
		std::cerr << "ERROR - Swissranger::GetCalibratedXYZ:" << std::endl;
		std::cerr << "\t ... fy is 0.\n";
		return RET_FAILED;
	}
	y = (float) (z*(v-cy)/fy); 

	/// Conversion from mm to m
	x /= 1000;
	y /= 1000;

	return RET_OK;
}

unsigned long Swissranger::GetCalibratedXYSwissranger(int u, int v, int width, float& x, float& y)
{
	// make sure, that m_X, m_Y and m_Z have been initialized by Acquire image
	int i = v*width + u;
	x = (float)m_X[i];
	y = (float)m_Y[i];

	return RET_OK;
}

unsigned long Swissranger::SetParameters()
{
	ipa_CameraSensors::t_cameraProperty cameraProperty;


/// -----------------------------------------------------------------
/// Set amplitude threshold
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_AMPLITUDE_THRESHOLD;
	std::string sAmplitudeThreshold = "";
	m_RangeCameraParameters.m_AmplitudeThreshold.clear(); /// Clear flags
	m_RangeCameraParameters.m_AmplitudeThreshold.seekg(0); /// Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_AmplitudeThreshold >> sAmplitudeThreshold;
	if (sAmplitudeThreshold == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sAmplitudeThreshold == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_SHORT);
		m_RangeCameraParameters.m_AmplitudeThreshold.clear(); /// Clear flags
		m_RangeCameraParameters.m_AmplitudeThreshold.seekg(0); /// Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_AmplitudeThreshold >> cameraProperty.u_shortData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - Swissranger::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set amplitude threshold" << std::endl;
	}

/// -----------------------------------------------------------------
/// Set integration time
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_INTEGRATION_TIME;
	std::string sIntegrationTime = "";
	m_RangeCameraParameters.m_IntegrationTime.clear(); /// Clear flags
	m_RangeCameraParameters.m_IntegrationTime.seekg(0); /// Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_IntegrationTime >> sIntegrationTime;
	if (sIntegrationTime == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sIntegrationTime == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		std::string tempValue;
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_CHARACTER);
		m_RangeCameraParameters.m_IntegrationTime.clear(); /// Clear flags
		m_RangeCameraParameters.m_IntegrationTime.seekg(0); /// Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_IntegrationTime >> tempValue;
		cameraProperty.u_charData = (unsigned char)atoi(tempValue.c_str());
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - Swissranger::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set integration time" << std::endl;
	}

/// -----------------------------------------------------------------
/// Set modulation frequency
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_MODULATION_FREQUENCY;
	std::string sModulationFrequency = "";
	m_RangeCameraParameters.m_ModulationFrequency.clear(); /// Clear flags
	m_RangeCameraParameters.m_ModulationFrequency.seekg(0); /// Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_ModulationFrequency >> sModulationFrequency;
	if (sModulationFrequency == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sModulationFrequency == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_STRING);
		m_RangeCameraParameters.m_ModulationFrequency.clear(); /// Clear flags
		m_RangeCameraParameters.m_ModulationFrequency.seekg(0); /// Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_ModulationFrequency >> cameraProperty.stringData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - Swissranger::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set modulation frequency" << std::endl;
	}

/// -----------------------------------------------------------------
/// Set acquire mode
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_ACQUIRE_MODE;
	cameraProperty.propertyType = ipa_CameraSensors::TYPE_INTEGER;
	m_RangeCameraParameters.m_AcquireMode.clear(); /// Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_AcquireMode.seekg(0); /// Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_AcquireMode >> cameraProperty.integerData;
	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - Swissranger::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set acquire mode" << std::endl;
	}

	return RET_OK;
}

unsigned long Swissranger::LoadParameters(const char* filename, int cameraIndex)
{
	/// Load SwissRanger parameters.
	TiXmlDocument* p_configXmlDocument = new TiXmlDocument( filename );
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file \n";
		std::cerr << "\t ... (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - Swissranger::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << filename << "'" << std::endl;

	std::string tempString;
	if ( p_configXmlDocument )
	{

//************************************************************************************
//	BEGIN LibCameraSensors
//************************************************************************************
		// Tag element "LibCameraSensors" of Xml Inifile		
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "LibCameraSensors" );

		if ( p_xmlElement_Root )
		{

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger
//************************************************************************************
			// Tag element "Swissranger3000" of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_SR31 = NULL;
			std::stringstream ss;
			ss << "Swissranger_" << cameraIndex;
			p_xmlElement_Root_SR31 = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_SR31 )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->Role
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "Role" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Role'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_RangeCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_RangeCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}

				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Role'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
				
//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->Interface
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "Interface" );
				std::string tempString;
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Interface'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "USB")
					{
						m_RangeCameraParameters.m_Interface.str( " " );	/// Clear stringstream
						m_RangeCameraParameters.m_Interface.clear();		/// Reset flags
						m_RangeCameraParameters.m_Interface << tempString;
					}
					else if (tempString == "ETHERNET")
					{
						m_RangeCameraParameters.m_Interface.str( " " );	/// Clear stringstream
						m_RangeCameraParameters.m_Interface.clear();		/// Reset flags
						m_RangeCameraParameters.m_Interface << tempString;
						// read and save value of attribute
						if ( p_xmlElement_Child->QueryValueAttribute( "ip", &tempString ) != TIXML_SUCCESS)
						{
							std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
							std::cerr << "\t ... Can't find attribute 'ip' of tag 'Interface'." << std::endl;
							return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
						}
						m_RangeCameraParameters.m_IP.str( " " );	/// Clear stringstream
						m_RangeCameraParameters.m_IP.clear();		/// Reset flags
						m_RangeCameraParameters.m_IP << tempString;
					}
					else
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Interface " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Interface'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			
//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->AmplitudeThreshold
//************************************************************************************
				// Subtag element "IntegrationTime" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "AmplitudeThreshold" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'AmplitudeThreshold'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_AmplitudeThreshold.str( " " );	/// Clear stringstream
						m_RangeCameraParameters.m_AmplitudeThreshold.clear();		/// Reset flags
						m_RangeCameraParameters.m_AmplitudeThreshold << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'AmplitudeThreshold'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->IntegrationTime
//************************************************************************************
				// Subtag element "IntegrationTime" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "IntegrationTime" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'IntegrationTime'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_IntegrationTime.str( " " );	/// Clear stringstream
						m_RangeCameraParameters.m_IntegrationTime.clear();		/// Reset flags
						m_RangeCameraParameters.m_IntegrationTime << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'IntegrationTime'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->Modulation
//************************************************************************************
				// Subtag element "IntegrationTime" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "Modulation" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "frequency", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'frequency' of tag 'Modulation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_ModulationFrequency.str( " " );	/// Clear stringstream
						m_RangeCameraParameters.m_ModulationFrequency.clear();		/// Reset flags
						m_RangeCameraParameters.m_ModulationFrequency << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Modulation'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->AcquireMode
//************************************************************************************
				// Subtag element "IntegrationTime" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "AcquireMode" );
				if ( p_xmlElement_Child )
				{
					int acquireMode = 0;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "AM_COR_FIX_PTRN", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_COR_FIX_PTRN' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_COR_FIX_PTRN;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_MEDIAN", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_MEDIAN' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_MEDIAN;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_TOGGLE_FRQ", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_TOGGLE_FRQ' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_TOGGLE_FRQ;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_CONV_GRAY", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_CONV_GRAY' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_CONV_GRAY;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_SW_ANF", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_SW_ANF' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_SW_ANF;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_SR3K_2TAP_PROC", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_SR3K_2TAP_PROC' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						//if (tempString == "ON") acquireMode |= AM_RESERVED0;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_SHORT_RANGE", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_SHORT_RANGE' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						//if (tempString == "ON") acquireMode |= AM_RESERVED1;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_CONF_MAP", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_CONF_MAP' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_CONF_MAP;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_HW_TRIGGER", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_HW_TRIGGER' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_HW_TRIGGER;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_SW_TRIGGER", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_SW_TRIGGER' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_SW_TRIGGER;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_DENOISE_ANF", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_DENOISE_ANF' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_DENOISE_ANF;
					}

					if ( p_xmlElement_Child->QueryValueAttribute( "AM_MEDIANCROSS", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'AM_MEDIANCROSS' of tag 'AcquireMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						if (tempString == "ON") acquireMode |= AM_MEDIANCROSS;
					}

					m_RangeCameraParameters.m_AcquireMode.str( " " );	/// Clear stringstream
					m_RangeCameraParameters.m_AcquireMode.clear();		/// Reset flags
					m_RangeCameraParameters.m_AcquireMode << acquireMode;
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'AcquireMode'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->CalibrationMethod
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "CalibrationMethod" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "name", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'name' of tag 'CalibrationMethod'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "MATLAB") m_CalibrationMethod = MATLAB;
					else if (tempString == "MATLAB_NO_Z") m_CalibrationMethod = MATLAB_NO_Z;
					else if (tempString == "NATIVE") m_CalibrationMethod = NATIVE;
					else
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CalibrationMethod'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			
//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->IntrinsicParameters
//************************************************************************************
				// Subtag element "IntrinsicParameters" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "IntrinsicParameters" );
				if ( p_xmlElement_Child )
				{
					double fx, fy, cx, cy;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "fx", &fx ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'fx' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "fy", &fy ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'fy' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "cx", &cx ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'cx' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "cy", &cy ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'cy' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					SetIntrinsicParameters(fx, fy, cx, cy);	
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'IntrinsicParameters'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Swissranger->DistortionCoeffs
//************************************************************************************
				// Subtag element "DistortionCoeffs " of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "DistortionCoeffs" );
				if ( p_xmlElement_Child )
				{
					double k1, k2, p1, p2;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "k1", &k1 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'k1' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "k2", &k2 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'k2' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "p1", &p1 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'p1' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "p2", &p2 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'p2' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					SetDistortionParameters(k1, k2, p1, p2, 176, 144);	
				}
				else
				{
					std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'DistortionCoeffs '." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}
//************************************************************************************
//	END LibCameraSensors->Swissranger
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - Swissranger::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	
	std::cout << "INFO - Swissranger::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml calibration file: Done.\n";

	return RET_OK;
}
