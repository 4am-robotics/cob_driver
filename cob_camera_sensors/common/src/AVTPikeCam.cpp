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
#include "cob_camera_sensors/AVTPikeCam.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AVTPikeCam.h"
#endif

using namespace std;
using namespace ipa_CameraSensors;

#ifdef __cplusplus
extern "C" {
#endif
__DLL_ABSTRACTCOLORCAMERA_H__ AbstractColorCamera* APIENTRY CreateColorCamera_AVTPikeCam()

{
	return (new AVTPikeCam());
}
#ifdef __cplusplus
}
#endif

bool AVTPikeCam::m_CloseExecuted = false;
bool AVTPikeCam::m_OpenExecuted = false;


AVTPikeCam::AVTPikeCam()
{
	m_initialized = false;
	m_open = false;
	m_BufferSize = 3;

#ifdef __LINUX__
	m_cam = 0;
	m_IEEE1394Cameras = 0;
	m_IEEE1394Info = 0;
	m_Frame = 0;
#endif
#ifndef __LINUX__
	m_Frame.pData = 0;
#endif
 					
}

AVTPikeCam::~AVTPikeCam()
{
	if (isOpen())
	{
		Close();
	}

#ifdef __LINUX__
	if (m_IEEE1394Info != 0)
	{
		dc1394_free(m_IEEE1394Info);
		m_IEEE1394Info = 0;
	}
#else
	// Close module and frees memory allocated by FireGrab
	if (m_ColorCameraParameters.m_CameraRole == ipa_CameraSensors::MASTER &&
		m_CloseExecuted == false)
	{
		FGExitModule();
		m_CloseExecuted = true;
	}
#endif
}


unsigned long AVTPikeCam::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}
	m_initialized = false;
	m_CameraType = ipa_CameraSensors::CAM_AVTPIKE;

	#ifdef __LINUX__

		/// Load parameters from xml initialization file
		std::string iniFileNameAndPath = directory + "cameraSensorsIni.xml";
		if (LoadParameters(iniFileNameAndPath.c_str(), cameraIndex) & RET_FAILED)
		{
			return (RET_FAILED | RET_INIT_CAMERA_FAILED);
		}


		/// Search for available cameras
		if (m_IEEE1394Info == 0)
		{
			m_IEEE1394Info = dc1394_new();
		}
		dc1394error_t err = dc1394_camera_enumerate(m_IEEE1394Info, &m_IEEE1394Cameras);                              
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::Init:" << std::endl;
			std::cerr << "\t ... Failed to enumerate cameras" << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}             

		if (m_IEEE1394Cameras->num == 0) 
		{                                                  
			std::cerr << "ERROR - AVTPikeCam::Init:" << std::endl;
			std::cerr << "\t ... No cameras found" << std::endl;
			return RET_FAILED;
		}

		m_initialized = true;
		return RET_OK;

	#else

		std::string iniFileNameAndPath = directory + "cameraSensorsIni.xml";
		if (LoadParameters(iniFileNameAndPath.c_str(), cameraIndex) & RET_FAILED)
		{
			return (RET_FAILED | RET_INIT_CAMERA_FAILED);
		}

		UINT32 err; ///< Error code variable

		/// Prepare FireGrab library for use
		if (m_OpenExecuted == false)
		{
			m_OpenExecuted = true;
			UINT32 err = FGInitModule(NULL);
			if(err!=FCE_NOERROR) /// err=1001 means library is already initialized (i.e. by other Pike camera)
			{
				std::cerr << "ERROR - AVTPikeCam::Init:" << std::endl;
				std::cerr << "\t ... Initialization of FireGrab library failed. ( error " << err << " )" << std::endl;
				return RET_FAILED;
			}
		}

		/// Retrieve a list of all nodes currently connected to the system
		/// At most 5 nodes are retrieved. Actual number is stored in nodeCnt
		m_NodeCnt = 0;
		err=FGGetNodeList(m_nodeInfo, 5, &m_NodeCnt);
		if(err!=FCE_NOERROR)
		{
			std::cerr << "ERROR - AVTPikeCam::Init:" << std::endl;
			std::cerr << "\t ... Retrival of connected IEEE1394 nodes failed. ( error " << err << " )" << std::endl;
			return RET_FAILED;
		}

		if (m_NodeCnt <= 0) 
		{                                                  
			std::cout << "ERROR - AVTPikeCam::Init:" << std::endl;
			std::cerr << "\t ... No cameras found." << std::endl;
			return RET_FAILED;
		}
		else
		{
			std::cout << "INFO - AVTPikeCam::Init:" << std::endl;
			std::cout << "\t ... Detected '" << m_NodeCnt << "' IEEE1394 node(s)" << std::endl;
			for(unsigned int i=0; i < m_NodeCnt; i++)
			{
				printf ("\t ... GUID of node '%i' is %-#08lX %-#08lX \n", i+1, m_nodeInfo[i].Guid.High, m_nodeInfo[i].Guid.Low);
			}
		}

		m_initialized = true;
		return RET_OK;

	#endif
}


unsigned long AVTPikeCam::Open()
{
	if (!isInitialized())
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t ... Camera not initialized." << std::endl;
		return (RET_FAILED);
	}
	m_open = false;

  
#ifdef __LINUX__
	dc1394error_t err;
	
	/// Connect with IEEE1394 node
	unsigned int i=0;
	for(; i<m_IEEE1394Cameras->num; i++)
	{
		uint64_t high = m_GUID.High;
		uint64_t low = m_GUID.Low;
		uint64_t guid = (high << 32) + low;
		if (m_IEEE1394Cameras->ids[i].guid == guid) break; 
	}
	/// Check if specified GUID has been found
	if (i == m_IEEE1394Cameras->num)
	{
		printf ("ERROR - AVTPikeCam::Open: \n\t ... Could not detect specified camera GUID %#08lX %#08lX on IEEE1394 bus\n", m_GUID.High, m_GUID.Low);
		return RET_FAILED;
	}

	m_cam = dc1394_camera_new(m_IEEE1394Info, m_IEEE1394Cameras->ids[i].guid);
	if (m_cam == 0)
	{
		m_open = false;
		return RET_FAILED;
	}

	dc1394_camera_free_list(m_IEEE1394Cameras);

	/// Set Parameters
	if (SetParameters() & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t... Error while setting parameters" << std::endl;
		return RET_FAILED;
	}

	err = dc1394_capture_setup(m_cam, m_BufferSize, DC1394_CAPTURE_FLAGS_DEFAULT);
	/// Relase allocated bandwidth and retry
	if (err!=DC1394_SUCCESS) 
	{   
		std::cout << "INFO - AVTPikeCam::Open:" << std::endl;
		std::cout << "\t ... Releasing bandwdith and retrying to setup DMA capture" << std::endl;
		uint32_t bandwidth = -1;
		err = dc1394_video_get_bandwidth_usage(m_cam, &bandwidth);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
			std::cerr << "\t ... Failed to get bandwith usage of camera device" << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;
		}
		dc1394_iso_release_bandwidth(m_cam, bandwidth);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
			std::cerr << "\t ... Failed to relase requested bandwidth '" << bandwidth << "'" << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;
		}
		err = dc1394_capture_setup(m_cam, m_BufferSize, DC1394_CAPTURE_FLAGS_DEFAULT);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
			std::cerr << "\t ... Failed to setup cameras device" << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;
		}
	}


	/// Start transmission
	err=dc1394_video_set_transmission(m_cam, DC1394_ON);                  
	if (err!=DC1394_SUCCESS) 
	{    
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t ... 'dc1394_video_set_transmission' failed." << std::endl;
		std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
		return RET_FAILED;                                         
	} 

#else

	UINT32 err;

	/// Connect with IEEE1394 node
	unsigned int i=0;
	for(; i<m_NodeCnt; i++)
	{
		if (m_nodeInfo[i].Guid.Low - m_GUID.Low == 0 && 
			m_nodeInfo[i].Guid.High - m_GUID.High == 0) break;
	}
	/// Check if specified GUID has been found
	if (i == m_NodeCnt)
	{
		printf ("ERROR - AVTPikeCam::Open: \n\t ... Could not detect specified camera GUID %#08lX %#08lX on IEEE1394 bus\n", m_GUID.High, m_GUID.Low);
		return RET_FAILED;
	}

	err=m_cam.Connect(&m_GUID);
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t ... Could not connect to camera. ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}

	if (SetParameters() & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" <<  std::endl;
		std::cerr << "\t ... Could not set parameters. ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}

	err = m_cam.SetParameter(FGP_FRAMEBUFFERCOUNT, m_BufferSize);
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t ... Could not set DMA buffer size to '"<< m_BufferSize 
                  << "' ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}

	// Opens, prepares and activates the image capture logic on the PC side
	err=m_cam.OpenCapture();
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" <<  std::endl;
		std::cerr << "\t ... Could not start DMA logic ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}
	
	/// Start image acquisition
	err=m_cam.StartDevice();
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" <<  std::endl;
		std::cerr << "\t ... Could not start camera device ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}

#endif
	std::cout << "**************************************************" << std::endl;
	std::cout << "AVTPikeCam::Open: AVT Pike 145C camera device OPEN" << std::endl;
	std::cout << "**************************************************" << std::endl << std::endl;
	m_open = true;
	return RET_OK;
}


unsigned long AVTPikeCam::SaveParameters(const char* filename)
{ 
	return RET_FAILED;
}





unsigned long AVTPikeCam::Close()
{
	if (!isOpen())
	{
		return (RET_OK);
	}

#ifdef __LINUX__
	if (m_cam != 0)
	{
		/// Stop transmission
		dc1394error_t err;
	    err=dc1394_video_set_transmission(m_cam, DC1394_OFF);                
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::Close:" << std::endl;
			std::cerr << "\t ... 'dc1394_video_set_transmission' failed." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		} 
	    dc1394_capture_stop(m_cam);                                      
		dc1394_camera_free(m_cam);
		m_cam = 0;
	}
#else
	/// Stops image acquisition
	UINT32 err;
	err=m_cam.StopDevice();
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Close:" << std::endl;
		std::cerr << "\t ... Could not stop camera device ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}

	/// Close capture logic and frees image buffers.
	/// Also frees all allocated resource on the FireWire bus.
	err=m_cam.CloseCapture();
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Close:" << std::endl;
		std::cerr << "\t ...  Could close capture logic. ( error " << err << " )" << std::endl;		return RET_FAILED;
		return RET_FAILED;
	}

	/// Disonnect object from external IEEE1394 node
	err=m_cam.Disconnect();
	if(err!=FCE_NOERROR)
	{
		std::cerr << "ERROR - AVTPikeCam::Close:" << std::endl;
		std::cerr << "\t ...  Could not close capture logic. ( error " << err << " )" << std::endl;		return RET_FAILED;
		return RET_FAILED;
	}

	
#endif

	m_open = false;
	return RET_OK;
} 



unsigned long AVTPikeCam::SetPropertyDefaults() 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long AVTPikeCam::GetProperty(t_cameraProperty* cameraProperty)
{
#ifdef __LINUX__
	dc1394error_t err;
	switch (cameraProperty->propertyID)
	{
		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;
		case PROP_CAMERA_RESOLUTION:	
			if (isOpen())
			{	
				uint32_t imageWidth = -1;
				uint32_t imageHeight = -1;
				dc1394video_mode_t videoMode;
				err=dc1394_video_get_mode(m_cam, &videoMode);
				if (err!=DC1394_SUCCESS) 
				{    
					std::cerr << "ERROR - AVTPikeCam::GetProperty:" << std::endl;
					std::cerr << "\t ... Failed to get video mode." << std::endl;
					std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
					return RET_FAILED;                                         
				} 
	
				err = dc1394_get_image_size_from_video_mode(m_cam, videoMode, &imageWidth, &imageHeight);
				if (err!=DC1394_SUCCESS) 
				{    
					std::cerr << "ERROR - AVTPikeCam::GetProperty:" << std::endl;
					std::cerr << "\t ... Failed to get image size." << std::endl;
					std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
					return RET_FAILED;                                         
				} 
				cameraProperty->cameraResolution.xResolution = (int) imageWidth;
				cameraProperty->cameraResolution.yResolution = (int) imageHeight;
			}
			else
			{
				std::cout << "WARNING - AVTPikeCam::GetProperty:" << std::endl;
				std::cout << "\t ... Camera not open" << std::endl;
				std::cout << "\t ... Resetting width and height to '1388'x'1038'" << std::endl;
				cameraProperty->cameraResolution.xResolution = 1388;
				cameraProperty->cameraResolution.yResolution = 1038;
				cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			}

			return RET_OK;
			break;
		default: 				
			std::cerr << "ERROR - AVTPikeCam::GetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified" << std::endl;
			return RET_FAILED;
			break;
	}
	return RET_OK;
#endif
#ifndef __LINUX__
	UINT32 err;
	switch (cameraProperty->propertyID)
	{
		case PROP_BRIGHTNESS:	
			
			break;
		case PROP_EXPOSURE_TIME:
			
			break;
		case PROP_WHITE_BALANCE_U:
			
			break;
		case PROP_WHITE_BALANCE_V:	
			
			break;
		case PROP_HUE:	
			
			break;
		case PROP_SATURATION:	
			
			break;
		case PROP_GAMMA:	
			
			break;
		case PROP_GAIN:	
			
			break;
		case PROP_CAMERA_RESOLUTION:	
			if (isOpen())
			{	
				UINT32 x;
				UINT32 y;
	
				err = m_cam.GetParameter(FGP_XSIZE, &x);
				if(err!=FCE_NOERROR)
				{
					std::cerr << "ERROR - AVTPikeCam::GetProperty:" << std::endl;
					std::cerr << "\t ... Could not read image width ( error " << err << " )" << std::endl;
					return RET_FAILED;
				}
				m_cam.GetParameter(FGP_YSIZE, &y);
				if(err!=FCE_NOERROR)
				{
					std::cerr << "ERROR - AVTPikeCam::GetProperty:" << std::endl;
					std::cerr << "\t ...  Could not read image height ( error " << err << " )" << std::endl;
					return RET_FAILED;
				}
				cameraProperty->cameraResolution.xResolution = x;
				cameraProperty->cameraResolution.yResolution = y;
			}
			else
			{
				std::cout << "WARNING - AVTPikeCam::GetProperty:" << std::endl;
				std::cout << "\t ... Camera not open" << std::endl;
				std::cout << "\t ... Resetting width and height to '1388'x'1038'" << std::endl;
				cameraProperty->cameraResolution.xResolution = 1388;
				cameraProperty->cameraResolution.yResolution = 1038;
				cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			}
	
			return RET_OK;
			break;
		case PROP_FW_OPERATION_MODE:
			cameraProperty->propertyType = TYPE_STRING;
			if (m_operationMode_B == false) cameraProperty->stringData == "A";
			else if (m_operationMode_B == true) cameraProperty->stringData == "B";
			break;
		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;
		case PROP_ISO_SPEED:
			
			break;
		default: 				
			std::cerr << "ERROR - AVTPikeCam::GetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified";
			return RET_FAILED;
			break;

	}

	return RET_OK;
#endif
} 


unsigned long AVTPikeCam::GetColorImage(char* colorImageData, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}
#ifdef __LINUX__
	
	dc1394error_t err;

	if (m_Frame)
	{
		/// Release the buffer from previous function call
		err=dc1394_capture_enqueue(m_cam, m_Frame);                            
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "AVTPikeCam::GetColorImage:" << std::endl;
			std::cerr << "\t ... 'dc1394_capture_enqueue' failed." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}
	}

	//std::cout << "INFO - AVTPikeCam::GetColorImage:" << std::endl;
	//std::cout << "\t ... Flushing DMA" << std::endl;

	if (getLatestFrame)
	{
		/// Flush the DMA ring buffer
		do
		{
			m_Frame = 0;
			err=dc1394_capture_dequeue(m_cam, DC1394_CAPTURE_POLICY_POLL, &m_Frame);
			if (err!=DC1394_SUCCESS) 
			{    
				std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
				std::cerr << "\t ... 'dc1394_capture_dequeue' failed." << std::endl;
				std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
				return RET_FAILED;                                         
			}

			if (m_Frame == 0)
			{
				break;
			}
			
			err=dc1394_capture_enqueue(m_cam, m_Frame);                            
			if (err!=DC1394_SUCCESS) 
			{    
				std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
				std::cerr << "\t ... 'dc1394_capture_enqueue' failed." << std::endl;
				std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
				return RET_FAILED;                                         
			}
		} 
		while (true);
	}

	//std::cout << "INFO - AVTPikeCam::GetColorImage:" << std::endl;
	//std::cout << "\t ... Waiting for images" << std::endl;

	/// Capture
	err=dc1394_capture_dequeue(m_cam, DC1394_CAPTURE_POLICY_WAIT, &m_Frame);
	if (err!=DC1394_SUCCESS) 
	{    
		std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... 'dc1394_capture_dequeue' failed." << std::endl;
		std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
		return RET_FAILED;                                         
	} 
	unsigned char * src = (unsigned char *)m_Frame->image;
	unsigned char * dst = (unsigned char *)colorImageData;
	
	/// Convert RGB to BGR
	int width;
	int height;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	if (GetProperty(&cameraProperty) & RET_FAILED) return RET_FAILED;
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	for (int i=0;i<width*height*3;i+=6) 
	{
		dst[i]   = src[i+2];
		dst[i+1] = src[i+1];
		dst[i+2] = src[i];
		dst[i+3] = src[i+5];
		dst[i+4] = src[i+4];
		dst[i+5] = src[i+3];
	}

	return RET_OK;
	
#else
	
	UINT32 err;

	/// Release previously acquired m_Frame
	if (m_Frame.pData != 0)
	{
		// Return m_Frame to module
		err=m_cam.PutFrame(&m_Frame);
		if(err!=FCE_NOERROR)
		{
			std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
			std::cerr << "\t ... Could not release image buffer ( error " << err << " )" << std::endl;
			return RET_FAILED;
		}
	}
		
	if (getLatestFrame)
	{
		/// One shot mode
		/// Reset image buffer to guarantee, that the latest images are returned 
		err = m_cam.DiscardFrames();
		if(err!=FCE_NOERROR)
		{
			std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
			std::cerr << "\t ... Could not reset image buffers ( error " << err << " )" << std::endl;
			return RET_FAILED;
		}
	}

	
	/// Blocking-Wait for the next m_Frame
	err = m_cam.GetFrame(&m_Frame);
	if(err!=FCE_NOERROR)
	{
		std::cerr << "AVTPikeCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Could not acquire image ( error " << err << " )" << std::endl;
		return RET_FAILED;
	}

	unsigned char * src = (unsigned char *)m_Frame.pData;
	unsigned char * dst = (unsigned char *)colorImageData;
	
	/// Convert RGB to BGR
	int width;
	int height;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	if (GetProperty(&cameraProperty) & RET_FAILED) return RET_FAILED;
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	for (int i=0; i<width*height*3; i+=6) 
	{
		dst[i]   = src[i+2];
		dst[i+1] = src[i+1];
		dst[i+2] = src[i];
		dst[i+3] = src[i+5];
		dst[i+4] = src[i+4];
		dst[i+5] = src[i+3];
	}
	
	return RET_OK;
	
#endif
}


unsigned long AVTPikeCam::GetColorImage(IplImage* colorImage, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int width;
	int height;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	if (GetProperty(&cameraProperty) & RET_FAILED) return RET_FAILED;
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;


	if(colorImage->depth == IPL_DEPTH_8U &&
		colorImage->nChannels == 3 &&
		colorImage->width == width &&
		colorImage->height == height)
	{
		return GetColorImage(colorImage->imageData, getLatestFrame);
	}
	else
	{
		std::cerr << "ERROR - AVTPikeCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Could not acquire color image. IplImage initialized with wrong attributes." << std::endl;
		return RET_FAILED;
	}

	return RET_FAILED;
}

unsigned long AVTPikeCam::GetColorImage2(IplImage** colorImage, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - AVTPikeCam::GetColorImage2" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int width;
	int height;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	if (GetProperty(&cameraProperty) & RET_FAILED) return RET_FAILED;
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	*colorImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	return GetColorImage((*colorImage)->imageData, getLatestFrame);
	
	return RET_FAILED;
}


unsigned long AVTPikeCam::PrintCameraInformation()
{
#ifndef __LINUX__
	FGPINFO info;
	string parameterName;
	std::cout << "INFO - AVTPikeCam::PrintCameraInformation:" << std::endl;

	for (int ID = FGP_BRIGHTNESS; ID <= FGP_BRIGHTNESS+10; ID++)
	{
		switch (ID)
		{
			case 0: parameterName = "FGP_IMAGEFORMAT"; break;		// Compact image format
			case 1: parameterName = "FGP_ENUMIMAGEFORMAT"; break;	// Enumeration (Reset,Get)
			case 2: parameterName = "FGP_BRIGHTNESS"; break;         // Set image brightness
			case 3: parameterName = "FGP_AUTOEXPOSURE"; break;		// Set auto exposure
			case 4: parameterName = "FGP_SHARPNESS"; break;          // Set image sharpness
			case 5: parameterName = "FGP_WHITEBALCB"; break;         // Blue
			case 6: parameterName = "FGP_WHITEBALCR"; break;         // Red
			case 7: parameterName = "FGP_HUE"; break;				// Set image hue
			case 8: parameterName = "FGP_SATURATION"; break;			// Set color saturation
			case 9: parameterName = "FGP_GAMMA"; break;				// Set gamma
			case 10: parameterName = "FGP_SHUTTER"; break;			// Shutter time
			case 11: parameterName = "FGP_GAIN"; break;				// Gain
			case 12: parameterName = "FGP_IRIS"; break;				// Iris
			case 13: parameterName = "FGP_FOCUS"; break;				// Focus
			case 14: parameterName = "FGP_TEMPERATURE"; break;		// Color temperature
			case 15: parameterName = "FGP_TRIGGER"; break;			// Trigger
			case 16: parameterName = "FGP_TRIGGERDLY"; break;		// Delay of trigger
			case 17: parameterName = "FGP_WHITESHD"; break;			// Whiteshade
			case 18: parameterName = "FGP_FRAMERATE"; break;			// Frame rate
			case 19: parameterName = "FGP_ZOOM"; break;				// Zoom
			case 20: parameterName = "FGP_PAN"; break;				// Pan
			case 21: parameterName = "FGP_TILT"; break;				// Tilt
			case 22: parameterName = "FGP_OPTICALFILTER"; break;		// Filter
			case 23: parameterName = "FGP_CAPTURESIZE"; break;		// Size of capture
			case 24: parameterName = "FGP_CAPTUREQUALITY"; break;	// Quality
			case 25: parameterName = "FGP_PHYSPEED"; break;			// Set speed for asy/iso
			case 26: parameterName = "FGP_XSIZE"; break;				// Image XSize
			case 27: parameterName = "FGP_YSIZE"; break;				// Image YSize
			case 28: parameterName = "FGP_XPOSITION"; break;			// Image x position
			case 29: parameterName = "FGP_YPOSITION"; break;			// Image y position
			case 30: parameterName = "FGP_PACKETSIZE"; break;		// Packet size
			case 31: parameterName = "FGP_DMAMODE"; break;			// DMA mode (continuous or limp)
			case 32: parameterName = "FGP_BURSTCOUNT"; break;		// Number of images to produce
			case 33: parameterName = "FGP_FRAMEBUFFERCOUNT"; break;	// Number of frame buffers
			case 34: parameterName = "FGP_USEIRMFORBW"; break;		// Allocate bandwidth or not (IsoRscMgr)
			case 35: parameterName = "FGP_ADJUSTPARAMETERS"; break;	// Adjust parameters or fail
			case 36: parameterName = "FGP_STARTIMMEDIATELY"; break;	// Start bursting immediately
			case 37: parameterName = "FGP_FRAMEMEMORYSIZE"; break;	// Read only: Frame buffer size
			case 38: parameterName = "FGP_COLORFORMAT"; break;		// Read only: Colorformat
			case 39: parameterName = "FGP_IRMFREEBW"; break;			// Read only: Free iso bytes for 400MBit
			case 40: parameterName = "FGP_DO_FASTTRIGGER"; break;	// Fast trigger (no ACK)
			case 41: parameterName = "FGP_DO_BUSTRIGGER"; break;		// Broadcast trigger
			case 42: parameterName = "FGP_RESIZE"; break;			// Start/Stop resizing
			case 43: parameterName = "FGP_USEIRMFORCHN"; break;		// Get channel over isochronous resource manager
			case 44: parameterName = "FGP_CAMACCEPTDELAY"; break;	// Delay after writing values
			case 45: parameterName = "FGP_ISOCHANNEL"; break;		// Iso channel
			case 46: parameterName = "FGP_CYCLETIME"; break;			// Read cycle time
			case 47: parameterName = "FGP_DORESET"; break;			// Reset camera
			case 48: parameterName = "FGP_DMAFLAGS"; break;			// Flags for ISO DMA
			case 49: parameterName = "FGP_R0C"; break;				// Ring 0 call gate
			case 50: parameterName = "FGP_BUSADDRESS"; break;		// Exact bus address
			case 51: parameterName = "FGP_CMDTIMEOUT"; break;		// Global bus command timeout
			case 52: parameterName = "FGP_CARD"; break;				// Card number of this camera (set before connect)
			case 53: parameterName = "FGP_LICENSEINFO"; break;		// Query license information
			case 54: parameterName = "FGP_PACKETCOUNT"; break;		// Read only: Packet count
			case 55: parameterName = "FGP_DO_MULTIBUSTRIGGER"; break;// Do trigger on several busses
			case 56: parameterName = "FGP_LAST"; break;
			
			default: parameterName = "Unknown parameter";
		}

		m_cam.GetParameterInfo(ID, &info);
		std::cout << parameterName << std::endl;
		std::cout << "\t ... Value: " << info.IsValue << std::endl;
		std::cout << "\t ... Max: " << info.MaxValue << std::endl;
		std::cout << "\t ... Min: " << info.MinValue << std::endl;
		std::cout << "\t ... Unit: " << info.Unit << std::endl;
		if (info.Specific.Data.FeatureInfo.AutoCap)
			std::cout << "\t ... Auto state: " << (bool) info.Specific.Data.FeatureInfo.AutoState << std::endl;
		std::cout << "\n" << std::endl;
	}
#endif
	return RET_OK;
}

unsigned long AVTPikeCam::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}

	return RET_OK;
}


unsigned long AVTPikeCam::SetProperty(t_cameraProperty* cameraProperty)

{
#ifdef __LINUX__
	dc1394error_t err;
#else
	UINT32 err;
	FGPINFO info;
#endif

	switch (cameraProperty->propertyID)
	{
///====================================================================
/// PROP_SHUTTER
///====================================================================
		case PROP_AMPLITUDE_THRESHOLD:
		case PROP_INTEGRATION_TIME:
		case PROP_VIDEO_MODE:
		case PROP_COLOR_MODE:
		case PROP_VIDEO_FORMAT:
		case PROP_CAMERA_RESOLUTION:
		case PROP_TIMEOUT:
		case PROP_REGISTER:
		case PROP_OPTICAL_FILTER:
		case PROP_SHARPNESS:
		case PROP_MODULATION_FREQUENCY:
			/// Not implemented
			break;
		case PROP_SHUTTER:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set shutter to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_SHUTTER, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set shutter time to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_SHUTTER, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Shutter time " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_SHUTTER, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set shutter time " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_SHUTTER, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Shutter time " << cameraProperty->u_longData 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_SHUTTER, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set shutter time " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_BRIGHTNESS
///====================================================================
		case PROP_BRIGHTNESS:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set brightness to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_BRIGHTNESS, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set brightness time to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_BRIGHTNESS, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Brighness " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_BRIGHTNESS, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set brighness " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_BRIGHTNESS, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Brigthness " << cameraProperty->u_longData << " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_BRIGHTNESS, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set brightness ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_EXPOSURE_TIME
///====================================================================
		case PROP_EXPOSURE_TIME:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set exposure time to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_AUTOEXPOSURE, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set exposure time to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_EXPOSURE, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Exposure time " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_EXPOSURE, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set exposure time " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_AUTOEXPOSURE, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Exposure time " << cameraProperty->u_longData 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_AUTOEXPOSURE, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set exposure time ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_WHITE_BALANCE_U
///====================================================================
		case PROP_WHITE_BALANCE_U:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set white balance U to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_WHITEBALCB, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set white balance U to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_WHITE_BALANCE, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... White balance U " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					uint32_t whiteU;
					uint32_t whiteV;
					dc1394_feature_whitebalance_get_value(m_cam, &whiteU, &whiteV);
					err=dc1394_feature_whitebalance_set_value(m_cam, (uint32_t) cameraProperty->u_longData, whiteV);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set white balance U " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_WHITEBALCB, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... White balance U value " << cameraProperty->u_longData 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_WHITEBALCB, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set white balance U value ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_WHITE_BALANCE_V
///====================================================================
		case PROP_WHITE_BALANCE_V:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_WHITE_BALANCE, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set white balance V to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_WHITEBALCR, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set white balance V to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_WHITE_BALANCE, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... white balance V " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					uint32_t whiteU;
					uint32_t whiteV;
					dc1394_feature_whitebalance_get_value(m_cam, &whiteU, &whiteV);
					err=dc1394_feature_whitebalance_set_value(m_cam, whiteU, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set white balance V " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_WHITEBALCR, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... White balance V value " << cameraProperty->u_longData 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_WHITEBALCR, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set white balance V value ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'unsigned int' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_HUE
///====================================================================
		case PROP_HUE:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_HUE, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set hue to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_HUE, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set image hue to AUTO ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_HUE, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Hue " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_HUE, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set hue " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_HUE, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Hue " << cameraProperty->u_longData 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
				}
				else
				{
					err = m_cam.SetParameter(FGP_HUE, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set image hue to " << cameraProperty->u_longData 
						<< " ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_SATURATION
///====================================================================
		case PROP_SATURATION:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_SATURATION, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set saturation to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_SATURATION, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... << Could not set saturation to AUTO ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_SATURATION, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Saturation " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_SATURATION, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set saturation " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_SATURATION, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Saturation has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
				}
				else
				{
					err = m_cam.SetParameter(FGP_SATURATION, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set saturation ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_GAMMA
///====================================================================
		case PROP_GAMMA:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_GAMMA, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set gamma to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_GAMMA, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... << Could not set gamma to AUTO ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_GAMMA, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Gamma " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_GAMMA, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set gamma " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_GAMMA, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Gamma has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_GAMMA, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... << Could not set gamma to " << cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}	
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_GAIN
///====================================================================			
		case PROP_GAIN:		
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_feature_set_mode(m_cam, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to set gamma to AUTO mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_GAIN, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set gain (on auto mode) ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				uint32_t min;
				uint32_t max;
				dc1394_feature_get_boundaries(m_cam, DC1394_FEATURE_GAIN, &min, &max);
				if ((cameraProperty->u_longData < min) | (cameraProperty->u_longData > max))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Gamma " << cameraProperty->u_longData 
						<< " has to be a value between " << min << " and " << max << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err=dc1394_feature_set_value(m_cam, DC1394_FEATURE_GAIN, (uint32_t) cameraProperty->u_longData);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set gamma " 
							<< cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
				}
#else
				m_cam.GetParameterInfo(FGP_GAIN, &info);
				if ((cameraProperty->u_longData < info.MinValue) | (cameraProperty->u_longData > info.MaxValue))
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty: " << std::endl;
					std::cerr << "\t ... Gain has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					return RET_FAILED;
				}
				else
				{
					err = m_cam.SetParameter(FGP_GAIN, cameraProperty->u_longData);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty: " << std::endl;
						std::cerr << "\t ... Could not set gain to value " << cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// VIDEO FORMAT, VIDEO MODE and COLOR CODING
///====================================================================	
		case PROP_VIDEO_ALL:		
			if (cameraProperty->propertyType ==
				(ipa_CameraSensors::TYPE_VIDEO_FORMAT | 
				ipa_CameraSensors::TYPE_VIDEO_MODE | 
				ipa_CameraSensors::TYPE_COLOR_MODE))
			{
#ifdef __LINUX__
				dc1394color_coding_t colorMode;
				dc1394video_mode_t videoMode;
				switch(cameraProperty->videoFormat)
				{
				case FORMAT_0:
					switch(cameraProperty->videoMode)
					{
					case MODE_1:
						videoMode = DC1394_VIDEO_MODE_320x240_YUV422;
						break;
					case MODE_2:
						videoMode = DC1394_VIDEO_MODE_640x480_YUV411;
						break;
					case MODE_3:
						videoMode = DC1394_VIDEO_MODE_640x480_YUV422;
						break;
					case MODE_4:
					case MODE_DEFAULT:
						videoMode = DC1394_VIDEO_MODE_640x480_RGB8;
						break;
					case MODE_5:
						videoMode = DC1394_VIDEO_MODE_640x480_MONO8;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode unknown." << std::endl;
						return RET_FAILED;
					}

					err=dc1394_video_set_mode(m_cam, videoMode);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to setup video mode for format 0" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					}

					break;
				case FORMAT_1:
					switch(cameraProperty->videoMode)
					{
					case MODE_0:
						videoMode = DC1394_VIDEO_MODE_800x600_YUV422;
						break;
					case MODE_1:
						videoMode = DC1394_VIDEO_MODE_800x600_RGB8;
						break;
					case MODE_2:
						videoMode = DC1394_VIDEO_MODE_800x600_MONO8;
						break;
					case MODE_3:
						videoMode = DC1394_VIDEO_MODE_1024x768_YUV422;
						break;
					case MODE_4:
					case MODE_DEFAULT:
						videoMode = DC1394_VIDEO_MODE_1024x768_RGB8;
						break;
					case MODE_5:
						videoMode = DC1394_VIDEO_MODE_1024x768_MONO8;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode not supported." << std::endl;
						return RET_FAILED;
					}

					err=dc1394_video_set_mode(m_cam, videoMode);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to setup video for format 1" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					}

					break;
				case FORMAT_2:
					switch(cameraProperty->videoMode)
					{
					case MODE_0:
						videoMode = DC1394_VIDEO_MODE_1280x960_YUV422;
						break;
					case MODE_1:
					case MODE_DEFAULT:
						videoMode = DC1394_VIDEO_MODE_1280x960_RGB8;
						break;
					case MODE_2:
						videoMode = DC1394_VIDEO_MODE_1280x960_MONO8;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode not supported." << std::endl;
						return RET_FAILED;
					}

					err=dc1394_video_set_mode(m_cam, videoMode);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to setup video for format 1" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					}

					break;
				case FORMAT_DEFAULT:
				case FORMAT_7:
					switch(cameraProperty->colorMode)
					{
					case COLOR_YUV411:
						colorMode = DC1394_COLOR_CODING_YUV411;
						break;
					case COLOR_YUV422:
						colorMode =  DC1394_COLOR_CODING_YUV422;
						break;
					case COLOR_YUV444:
						colorMode =  DC1394_COLOR_CODING_YUV444;
						break;
					case COLOR_DEFAULT:
					case COLOR_RGB8:
						colorMode =  DC1394_COLOR_CODING_RGB8;
						break;
					case COLOR_RGB16S:
						colorMode =  DC1394_COLOR_CODING_RGB16S;
						break;
					case COLOR_RGB16:
						colorMode =  DC1394_COLOR_CODING_RGB16;
						break;
					case COLOR_MONO8:
						colorMode =  DC1394_COLOR_CODING_MONO8;
						break;
					case COLOR_MONO16S:
						colorMode =  DC1394_COLOR_CODING_MONO16S;
						break;
					case COLOR_MONO16:
						colorMode =  DC1394_COLOR_CODING_MONO16;
						break;
					case COLOR_RAW8:
						colorMode =  DC1394_COLOR_CODING_RAW8;
						break;
					case COLOR_RAW16:
						colorMode =  DC1394_COLOR_CODING_RAW16;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified color mode not supported." << std::endl;
						return RET_FAILED;
					}


					switch(cameraProperty->videoMode)
					{
					case MODE_DEFAULT:
					case MODE_0:
						videoMode = DC1394_VIDEO_MODE_FORMAT7_0;
						break;
					case MODE_4:
						videoMode = DC1394_VIDEO_MODE_FORMAT7_4;
						break;
					case MODE_5:
						videoMode = DC1394_VIDEO_MODE_FORMAT7_5;
						break;
					case MODE_6:
						videoMode = DC1394_VIDEO_MODE_FORMAT7_6;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode not supported." << std::endl;
						return RET_FAILED;
					}
					
					
					err = dc1394_video_set_mode(m_cam, videoMode);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to setup video mode." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
					err = dc1394_format7_set_color_coding(m_cam, videoMode, colorMode);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to setup color coding." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					}

					break;
				default:
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Specified video format unknown." << std::endl;
					return RET_FAILED;
				}
#else

				FG_RESOLUTION resolution;
				FG_COLORMODE colorMode;
				int videoMode = -1;
				switch(cameraProperty->videoFormat)
				{
				case FORMAT_0:
					switch(cameraProperty->videoMode)
					{
					case MODE_1:
						resolution = RES_320_240;
						colorMode = CM_YUV422;
						break;
					case MODE_2:
						resolution = RES_640_480;
						colorMode = CM_YUV411;
						break;
					case MODE_3:
						resolution = RES_640_480;
						colorMode = CM_YUV422;
						break;
					case MODE_4:
					case MODE_DEFAULT:
						resolution = RES_640_480;
						colorMode = CM_RGB8;
						break;
					case MODE_5:
						resolution = RES_640_480;
						colorMode = CM_Y8;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode unknown." << std::endl;
						return RET_FAILED;
					}

					err = m_cam.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(resolution, colorMode, FR_30));
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set video format and mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}

					break;
				case FORMAT_1:
					switch(cameraProperty->videoMode)
					{
					case MODE_0:
						resolution = RES_800_600;
						colorMode = CM_YUV422;
						break;
					case MODE_1:
						resolution = RES_800_600;
						colorMode = CM_RGB8;
						break;
					case MODE_2:
						resolution = RES_800_600;
						colorMode = CM_Y8;
						break;
					case MODE_3:
						resolution = RES_1024_768;
						colorMode = CM_YUV422;
						break;
					case MODE_4:
					case MODE_DEFAULT:
						resolution = RES_1024_768;
						colorMode = CM_RGB8;
						break;
					case MODE_5:
						resolution = RES_1024_768;
						colorMode = CM_Y8;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode not supported." << std::endl;
						return RET_FAILED;
					}

					err = m_cam.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(resolution, colorMode, FR_15));
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set video format and mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}

					break;
				case FORMAT_2:
					switch(cameraProperty->videoMode)
					{
					case MODE_0:
						resolution = RES_1280_960;
						colorMode = CM_YUV422;
						break;
					case MODE_1:
					case MODE_DEFAULT:
						resolution = RES_1280_960;
						colorMode = CM_RGB8;
						break;
					case MODE_2:
						resolution = RES_1280_960;
						colorMode = CM_Y8;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode not supported." << std::endl;
						return RET_FAILED;
					}

					err = m_cam.SetParameter(FGP_IMAGEFORMAT, MAKEIMAGEFORMAT(resolution, colorMode, FR_15));
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set video format and mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}

					break;
				case FORMAT_DEFAULT:
				case FORMAT_7:
					switch(cameraProperty->colorMode)
					{
					case COLOR_YUV444:
						colorMode = CM_YUV444;
						break;
					case COLOR_YUV422:
						colorMode =  CM_YUV422;
						break;
					case COLOR_YUV411:
						colorMode =  CM_YUV411;
						break;
					case COLOR_DEFAULT:
					case COLOR_RGB8:
						colorMode =  CM_RGB8;
						break;
					case COLOR_RGB16S:
						colorMode =  CM_SRGB16;
						break;
					case COLOR_RGB16:
						colorMode =  CM_RGB16;
						break;
					case COLOR_MONO8:
						colorMode =  CM_Y8;
						break;
					case COLOR_MONO16S:
						colorMode =  CM_SY16;
						break;
					case COLOR_MONO16:
						colorMode =  CM_Y16;
						break;
					case COLOR_RAW8:
						colorMode =  CM_RAW8;
						break;
					case COLOR_RAW16:
						colorMode =  CM_RAW16;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified color mode not supported." << std::endl;
						return RET_FAILED;
					}

					switch(cameraProperty->videoMode)
					{
					case MODE_0:
					case COLOR_DEFAULT:
						videoMode = 0;
						break;
					case MODE_4:
						videoMode = 4;
						break;
					case MODE_5:
						videoMode = 5;
						break;
					case MODE_6:
						videoMode = 6;
						break;
					default:
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified video mode not supported." << std::endl;
						return RET_FAILED;
					}

					err = m_cam.SetParameter(FGP_IMAGEFORMAT, MAKEDCAMFORMAT(RES_SCALABLE, videoMode, colorMode));
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set video format and mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}

					break;
				default:
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Specified video format not supported." << std::endl;
					return RET_FAILED;
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_VIDEO_MODE' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;

///====================================================================
/// PROP_FRAME_RATE
///====================================================================	
		case PROP_FRAME_RATE:
#ifdef __LINUX__
			dc1394video_mode_t videoMode;
			err=dc1394_video_get_mode(m_cam, &videoMode);
			if(err!=DC1394_SUCCESS)
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not get video mode ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
				return RET_FAILED;
			}
			
			/// Format 7
			/// Bytes_per_packet = (fps * width * height * ByteDepth * 125microsec)/1000000
			if (videoMode==DC1394_VIDEO_MODE_FORMAT7_0 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_1 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_2 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_3 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_4 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_5 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_6 ||
				videoMode==DC1394_VIDEO_MODE_FORMAT7_7)
			{
#else
			UINT32 imageFormat;
			FGPINFO packetSizeInfo;
			err = m_cam.GetParameter(FGP_IMAGEFORMAT, &imageFormat);
			if(err!=FCE_NOERROR)
			{
				std::cout << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Could not read image format ( error " << err << " )" << std::endl;
				return RET_FAILED;
			}
			
			/// Format 7
			/// Bytes_per_packet = (fps * width * height * ByteDepth * 125microsec)/1000000
			if (IMGRES(imageFormat)==RES_SCALABLE)
			{
#endif
				if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
				{
					if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO || 
						cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
					{
						/// Void
					}
				}
				else if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_FLOAT)
				{
#ifdef __LINUX__
					uint64_t bytesPerFrame;
					
					err = dc1394_format7_get_total_bytes(m_cam, videoMode, &bytesPerFrame);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not get bytes per image ( error " << err << " )" << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
					
					if (cameraProperty->floatData > 0 && cameraProperty->floatData <= 60)
					{
						double fps = cameraProperty->floatData;
						uint32_t min = 0;
						uint32_t max = 0;
						uint64_t bytesPerFrame = 0;
						uint32_t bytesPerPacket = 0;
						unsigned long packetsPerFrame = 0;
						double busPeriodInSeconds = 0;
						dc1394speed_t isoSpeed;

						err = dc1394_format7_get_packet_parameters(m_cam, videoMode, &min, &max);
						if (err!=DC1394_SUCCESS) 
						{    
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not get min/max bytes per packet ( error " << err << " )" << std::endl;
							std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
							return RET_FAILED;                                         
						} 
						err = dc1394_format7_get_total_bytes(m_cam, videoMode, &bytesPerFrame);
						if (err!=DC1394_SUCCESS) 
						{    
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not get total bytes per frame ( error " << err << " )" << std::endl;
							std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
							return RET_FAILED;                                         
						}

						err = dc1394_video_get_iso_speed(m_cam, &isoSpeed);
						if (err!=DC1394_SUCCESS) 
						{    
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not get iso speed ( error " << err << " )" << std::endl;
							std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
							return RET_FAILED;                                         
						}

						if (isoSpeed == DC1394_ISO_SPEED_100)
						{
							busPeriodInSeconds = 500.0/1000000.0;
						}
						else if(isoSpeed == DC1394_ISO_SPEED_200)
						{
							busPeriodInSeconds = 250.0/1000000.0;
						}
						else if(isoSpeed == DC1394_ISO_SPEED_400)
						{
							busPeriodInSeconds = 125.0/1000000.0;
						}
						else
						{
							busPeriodInSeconds = 62.5/1000000.0;
						}
						
						packetsPerFrame = (int) (1.0/(busPeriodInSeconds*fps) + 0.5);
						bytesPerPacket = (uint32_t) ((bytesPerFrame + packetsPerFrame - 1)/(packetsPerFrame));

						if (bytesPerPacket < min || bytesPerPacket > max)
						{
							std::cout << "WARNING - AVTPikeCam::SetProperty:" << std::endl;
							std::cout << "\t ... Desired packet size out of bounds. Resetting packet size to max value" << std::endl;
							bytesPerPacket = max;
						}

						/// Bytes per packet must be a multiple of min bytes
						bytesPerPacket = ((int)(bytesPerPacket/min))*min;

						err = dc1394_format7_set_packet_size(m_cam, videoMode, bytesPerPacket);
						if (err!=DC1394_SUCCESS) 
						{
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not set packet size " << bytesPerPacket << " ( error " << err << " )" << std::endl;
							return RET_FAILED;
						}
					}
#else
					UINT32 bytesPerImage = 0;
					UINT32 isoSpeed = 0;
					double busPeriodInSeconds = 0;
					err = m_cam.GetParameter(FGP_FRAMEMEMORYSIZE, &bytesPerImage);
					if(err!=FCE_NOERROR)
					{
						std::cout << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not read image size ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}

					err = m_cam.GetParameter(FGP_PHYSPEED, &isoSpeed);
					if (err!=FCE_NOERROR) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not read iso speed info ( error " << err << " )" << std::endl;
						return RET_FAILED;                                         
					}

					if (isoSpeed == PS_100MBIT)
					{
						busPeriodInSeconds = 500.0/1000000.0;
					}
					else if(isoSpeed == PS_200MBIT)
					{
						busPeriodInSeconds = 250.0/1000000.0;
					}
					else if(isoSpeed == PS_400MBIT)
					{
						busPeriodInSeconds = 125.0/1000000.0;
					}
					else
					{
						busPeriodInSeconds = 62.5/1000000.0;
					}
					
					if (cameraProperty->floatData > 0 && cameraProperty->floatData <= 60)
					{
						double fps = cameraProperty->floatData;
						UINT32 bytesPerPacket = (UINT32) (fps * busPeriodInSeconds * (double)bytesPerImage);

						err = m_cam.GetParameterInfo(FGP_PACKETSIZE, &packetSizeInfo);
						if(err!=FCE_NOERROR)
						{
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not read packet size info ( error " << err << " )" << std::endl;
							return RET_FAILED;
						}

						if (bytesPerPacket < packetSizeInfo.MinValue || bytesPerPacket > packetSizeInfo.MaxValue)
						{
							std::cout << "WARNING - AVTPikeCam::SetProperty:" << std::endl;
							std::cout << "\t ... Desired packet size out of bounds. Resetting packet size to max value" << std::endl;
							bytesPerPacket = packetSizeInfo.MaxValue;
						}
						
						/// Bytes per packet must be a multiple of min bytes
						bytesPerPacket = ((int)(bytesPerPacket/packetSizeInfo.MinValue))*packetSizeInfo.MinValue;

						err = m_cam.SetParameter(FGP_PACKETSIZE, bytesPerPacket);
						if(err!=FCE_NOERROR)
						{
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not set packet size " << bytesPerPacket << " ( error " << err << " )" << std::endl;
							return RET_FAILED;
						}
					}
#endif
					
					else 
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified framerate out of range ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Wrong property type. 'TYPE_FLOAT' or special values 'TYPE_SPECIAL' expected." << std::endl;
					return RET_FAILED;
				}
			}
			/// Other formats than Format 7
			else
			{
				if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
				{
					if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
					{
#ifdef __LINUX__
						err=dc1394_video_set_framerate(m_cam, DC1394_FRAMERATE_7_5);
						if (err!=DC1394_SUCCESS) 
						{    
							std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Failed to set framerate to AUTO mode." << std::endl;
							std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
							return RET_FAILED;                                         
						} 
#else
						err = m_cam.SetParameter(FGP_FRAMERATE, PVAL_AUTO);
						if(err!=FCE_NOERROR)
						{
							std::cout << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
							std::cerr << "\t ... Could not set framerate to AUTO ( error " << err << " )" << std::endl;
							return RET_FAILED;
						}
#endif
					}
					if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
					{
						/// Void
					}
					else
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
						return RET_FAILED;
					}
				}
				else if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_FLOAT)
				{
#ifdef __LINUX__
					dc1394framerate_t framerate;
					if (cameraProperty->floatData <= 1.875)
					{
						framerate = DC1394_FRAMERATE_1_875;
					}
					else if (cameraProperty->floatData <= 3.75)
					{
						framerate = DC1394_FRAMERATE_3_75;
					}
					else if (cameraProperty->floatData <= 7.5)
					{
						framerate = DC1394_FRAMERATE_7_5;
					}
					else if (cameraProperty->floatData <= 15)
					{
						framerate = DC1394_FRAMERATE_15;
					}		
					else if (cameraProperty->floatData <= 30)
					{		
						framerate = DC1394_FRAMERATE_30;
					}
					else if (cameraProperty->floatData <= 60)
					{
						framerate = DC1394_FRAMERATE_60;
					}
					else 
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified framerate " << cameraProperty->floatData << " out of range ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
					err=dc1394_video_set_framerate(m_cam, framerate);
					if (err!=DC1394_SUCCESS) 
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set specified framerate " << cameraProperty->floatData << " ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#else
					UINT32 framerate;
					if (cameraProperty->floatData <= 1.875)
					{
						framerate = FR_1_875;
					}
					else if (cameraProperty->floatData <= 3.75)
					{
						framerate = FR_3_75;
					}
					else if (cameraProperty->floatData <= 7.5)
					{
						framerate = FR_7_5;
					}
					else if (cameraProperty->floatData <= 15)
					{
						framerate = FR_15;
					}		
					else if (cameraProperty->floatData <= 30)
					{		
						framerate = FR_30;
					}
					else if (cameraProperty->floatData <= 60)
					{
						framerate = FR_60;
					}
					else 
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Specified framerate " << cameraProperty->floatData << " out of range ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
					err = m_cam.SetParameter(FGP_FRAMERATE, framerate);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set specified framerate " << cameraProperty->floatData << " ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Wrong property type. 'TYPE_FLOAT' or special values 'TYPE_SPECIAL' expected." << std::endl;
					return RET_FAILED;
				}
			}
			return RET_OK;
			break;
///====================================================================
/// PROP_FW_OPERATION_MODE
///====================================================================	
		case PROP_FW_OPERATION_MODE:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					m_operationMode_B = false;
					return RET_OK;
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					m_operationMode_B = false;
					return RET_OK;
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}

			}
			else if (cameraProperty->propertyType & TYPE_STRING)
			{
				if (cameraProperty->stringData == "A")
				{
					m_operationMode_B = false;
					return RET_OK;
				}
				else if (cameraProperty->stringData == "B") 
				{
					m_operationMode_B = true;
					return RET_OK;
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... FireWire operation mode " << cameraProperty->stringData << " unknown." << std::endl;
					return RET_FAILED;
				}
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_STRING' or special values 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_ISO_SPEED
///====================================================================	
		case PROP_ISO_SPEED:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
					err=dc1394_video_set_iso_speed(m_cam, DC1394_ISO_SPEED_400);
					if (err!=DC1394_SUCCESS) 
					{    
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Failed to setup iso speed." << std::endl;
						std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
						return RET_FAILED;                                         
					} 
#else
					err = m_cam.SetParameter(FGP_PHYSPEED, PS_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr<< "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set iso speed to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
#ifdef __LINUX__
#else
					err = m_cam.SetParameter(FGP_PHYSPEED, PS_LAST);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set iso speed to DEFAULT mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
#ifdef __LINUX__
				err = DC1394_FAILURE;
				if (cameraProperty->u_longData <= 100)
				{
					err=dc1394_video_set_iso_speed(m_cam, DC1394_ISO_SPEED_100);
				}
				else if (cameraProperty->u_longData <= 200)
				{
					err=dc1394_video_set_iso_speed(m_cam, DC1394_ISO_SPEED_200);
				}
				else if (cameraProperty->u_longData <= 400)
				{
					err=dc1394_video_set_iso_speed(m_cam, DC1394_ISO_SPEED_400);
				}
				else if (cameraProperty->u_longData <= 800)
				{
					err=dc1394_video_set_iso_speed(m_cam, DC1394_ISO_SPEED_800);
				}
				if (err!=DC1394_SUCCESS) 
				{    
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Failed to setup iso speed " << cameraProperty->u_longData << std::endl;
					std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
					return RET_FAILED;                                         
				} 
#else
				err = 1;
				if (cameraProperty->u_longData <= 100)
				{
					err = m_cam.SetParameter(FGP_PHYSPEED, PS_100MBIT);
				}
				else if (cameraProperty->u_longData <= 200)
				{
					err = m_cam.SetParameter(FGP_PHYSPEED, PS_200MBIT);
				}
				else if (cameraProperty->u_longData <= 400)
				{
					err = m_cam.SetParameter(FGP_PHYSPEED, PS_400MBIT);
				}
				else if (cameraProperty->u_longData <= 800)
				{
					err = m_cam.SetParameter(FGP_PHYSPEED, PS_800MBIT);
				}
				if(err!=FCE_NOERROR)
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set iso speed to " << 
						cameraProperty->u_longData << " ( error " << err << " )" << std::endl;
					return RET_FAILED;
				}
#endif
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// PROP_RESOLUTION
///====================================================================	
		case PROP_RESOLUTION:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
#ifdef __LINUX__
#else
					err = m_cam.SetParameter(FGP_XSIZE, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set x resolution to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
					err = m_cam.SetParameter(FGP_YSIZE, PVAL_AUTO);
					if(err!=FCE_NOERROR)
					{
						std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set y resolution to AUTO mode ( error " << err << " )" << std::endl;
						return RET_FAILED;
					}
#endif
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					/// Void
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_CAMERA_RESOLUTION)
			{
#ifdef __LINUX__
#else
				m_cam.GetParameterInfo(FGP_XSIZE, &info);
				if (((unsigned int) cameraProperty->cameraResolution.xResolution < info.MinValue) ||
					((unsigned int) cameraProperty->cameraResolution.xResolution > info.MaxValue))
				{
					std::cout << "WARNING - AVTPikeCam::SetProperty:" << std::endl;
					std::cout << "\t ... x resolution " << cameraProperty->cameraResolution.xResolution 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					std::cout << "\t ... Setting maximal x resolution" << std::endl;
				}
				m_cam.GetParameterInfo(FGP_YSIZE, &info);
				if (((unsigned int) cameraProperty->cameraResolution.yResolution < info.MinValue) ||
					((unsigned int) cameraProperty->cameraResolution.yResolution > info.MaxValue))
				{
					std::cout << "WARNING - AVTPikeCam::SetProperty:" << std::endl;
					std::cout << "\t ... y resolution " << cameraProperty->cameraResolution.yResolution 
						<< " has to be a value between " << info.MinValue << " and " << info.MaxValue << "." << std::endl;
					std::cout << "\t ... Setting maximal y resolution" << std::endl;
				}

				/// Set x/y resolution. If values are out of range, 
				/// the maximal possible reolution is set through the AVT library
				err = m_cam.SetParameter(FGP_XSIZE, cameraProperty->cameraResolution.xResolution);
				if(err!=FCE_NOERROR)
				{
					std::cout << "WARNING - AVTPikeCam::SetProperty:" << std::endl;
					std::cout << "\t ... Could not set image width ( error " << err << " )" << std::endl;
				}

				err = m_cam.SetParameter(FGP_YSIZE, cameraProperty->cameraResolution.yResolution);
				if(err!=FCE_NOERROR)
				{
					std::cout << "WARNING - AVTPikeCam::SetProperty:" << std::endl;
					std::cout << "\t ... Could not set image height ( error " << err << " )" << std::endl;
				}
#endif
				
			}
			else
			{
				std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_CAMERA_RESOLUTION' or 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
///====================================================================
/// DEFAULT
///====================================================================	
		default: 
			std::cerr << "ERROR - VTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified." << std::endl;
			return RET_FAILED;
			break;
	}

	return RET_OK;
}

unsigned long AVTPikeCam::SetParameters()
{
	ipa_CameraSensors::t_cameraProperty cameraProperty;

/// -----------------------------------------------------------------
/// Setup trigger
/// -----------------------------------------------------------------
	if (m_ColorCameraParameters.m_CameraRole == MASTER)
	{
		std::cout << "Info - AVTPikeCam::SetProperty:" << std::endl;
		std::cout << "\t ... Setting camera in MASTER mode." << std::endl;
#ifdef __LINUX__
		/// Trigger OFF, Trigger mode 3 -> Edge mode 0, falling
		/// Trigger OFF -> Edge mode 0, falling
		dc1394error_t err;
		err = dc1394_external_trigger_set_power(m_cam, DC1394_OFF);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Failed to deactivate external trigger." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}
		err = dc1394_external_trigger_set_mode(m_cam, DC1394_TRIGGER_MODE_3);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Failed to set trigger mode." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}
#else
		UINT32 err;
		/// Use internal trigger
		UINT32 triggerValue;
		triggerValue=MAKETRIGGER(0, 0, 0, 0, 0);
		err = m_cam.SetParameter(FGP_TRIGGER,triggerValue);
		if(err!=FCE_NOERROR)
		{
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Could not set Slave trigger mode ( error " << err << " )" << std::endl;
			return RET_FAILED;
		}
#endif
	}
	else if (m_ColorCameraParameters.m_CameraRole == SLAVE)
	{
		std::cout << "Info - AVTPikeCam::SetProperty:" << std::endl;
		std::cout << "\t ... Setting camera in SLAVE mode." << std::endl;
#ifdef __LINUX__
		/// Trigger ON, Trigger mode 0, low -> Edge mode 0, rizing
		dc1394error_t err;
		err = dc1394_external_trigger_set_power(m_cam, DC1394_ON);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Failed to activate external trigger." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}
		
		err = dc1394_external_trigger_set_polarity(m_cam, DC1394_TRIGGER_ACTIVE_HIGH);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Failed to set trigger polarity." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}

		err = dc1394_external_trigger_set_mode(m_cam, DC1394_TRIGGER_MODE_0);
		if (err!=DC1394_SUCCESS) 
		{    
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Failed to set trigger mode." << std::endl;
			std::cerr << "\t ... " << dc1394_error_get_string(err) << std::endl;
			return RET_FAILED;                                         
		}
#else
		UINT32 err;
		UINT32 nOn=1; // 0=ext. trigger off, 1=ext. trigger on
		UINT32 nPolarity=1; // 0=low active input, 1=high active input
		UINT32 nSrc=0; // not currently applicable to AVT cameras
		UINT32 nMode=0; // 0=edge mode, 1=level mode, 15=bulk mode
		UINT32 nParm=0; // not currently applicable to AVT cameras
		UINT32 triggerValue;
		triggerValue=MAKETRIGGER(nOn, nPolarity, nSrc, nMode, nParm);
		err = m_cam.SetParameter(FGP_TRIGGER,triggerValue);
		if(err!=FCE_NOERROR)
		{
			std::cerr << "ERROR - AVTPikeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Could not set Slave trigger mode ( error " << err << " )" << std::endl;
			return RET_FAILED;
		}
#endif
	}

/// -----------------------------------------------------------------
/// Set isochronous speed
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_ISO_SPEED;
	std::string sIsoSpeed = "";
	m_ColorCameraParameters.m_IsoSpeed.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_IsoSpeed.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_IsoSpeed >> sIsoSpeed;
	if (sIsoSpeed == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sIsoSpeed == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_IsoSpeed.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_IsoSpeed.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_IsoSpeed >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set ISO speed." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set video mode, video format and color mode
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_VIDEO_ALL;
	cameraProperty.propertyType = (ipa_CameraSensors::TYPE_VIDEO_FORMAT | 
				ipa_CameraSensors::TYPE_VIDEO_MODE | 
				ipa_CameraSensors::TYPE_COLOR_MODE);
	std::string sVideoFormat = "";
	std::string sVideoMode = "";
	std::string sColorMode = "";
	
	m_ColorCameraParameters.m_VideoFormat.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_VideoFormat.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_VideoFormat >> sVideoFormat;
	if (sVideoFormat == "FORMAT_DEFAULT")
	{
		cameraProperty.videoFormat = FORMAT_DEFAULT;
	}
	else if (sVideoFormat == "FORMAT_0")
	{
		cameraProperty.videoFormat = FORMAT_0;
	}
	else if (sVideoFormat == "FORMAT_1")
	{
		cameraProperty.videoFormat = FORMAT_1;
	}
	else if (sVideoFormat == "FORMAT_2")
	{
		cameraProperty.videoFormat = FORMAT_2;
	}
	else if (sVideoFormat == "FORMAT_7")
	{
		cameraProperty.videoFormat = FORMAT_7;
	}

	m_ColorCameraParameters.m_VideoMode.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_VideoMode.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_VideoMode >> sVideoMode;
	if (sVideoMode == "MODE_DEFAULT")
	{
		cameraProperty.videoMode = MODE_DEFAULT;
	}
	else if (sVideoMode == "MODE_0")
	{
		cameraProperty.videoMode = MODE_0;
	}
	else if (sVideoMode == "MODE_1")
	{
		cameraProperty.videoMode = MODE_1;
	}
	else if (sVideoMode == "MODE_2")
	{
		cameraProperty.videoMode = MODE_2;
	}
	else if (sVideoMode == "MODE_3")
	{
		cameraProperty.videoMode = MODE_3;
	}
	else if (sVideoMode == "MODE_4")
	{
		cameraProperty.videoMode = MODE_4;
	}
	else if (sVideoMode == "MODE_5")
	{
		cameraProperty.videoMode = MODE_5;
	}
	else if (sVideoMode == "MODE_6")
	{
		cameraProperty.videoMode = MODE_6;
	}
	else if (sVideoMode == "MODE_7")
	{
		cameraProperty.videoMode = MODE_7;
	}

	m_ColorCameraParameters.m_ColorMode.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_ColorMode.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_ColorMode >> sColorMode;
	if (sColorMode == "COLOR_DEFAULT")
	{
		cameraProperty.colorMode = COLOR_DEFAULT;
	}
	else if (sColorMode == "COLOR_YUV422")
	{
		cameraProperty.colorMode = COLOR_YUV422;
	}
	else if (sColorMode == "COLOR_YUV411")
	{
		cameraProperty.colorMode = COLOR_YUV411;
	}
	else if (sColorMode == "COLOR_RGB8")
	{
		cameraProperty.colorMode = COLOR_RGB8;
	}
	else if (sColorMode == "COLOR_RGB16S")
	{
		cameraProperty.colorMode = COLOR_RGB16S;
	}
	else if (sColorMode == "COLOR_RGB16")
	{
		cameraProperty.colorMode = COLOR_RGB16;
	}
	else if (sColorMode == "COLOR_MONO8")
	{
		cameraProperty.colorMode = COLOR_MONO8;
	}
	else if (sColorMode == "COLOR_MONO16")
	{
		cameraProperty.colorMode = COLOR_MONO16;
	}
	else if (sColorMode == "COLOR_MONO16S")
	{
		cameraProperty.colorMode = COLOR_MONO16S;
	}
	else if (sColorMode == "COLOR_RAW8")
	{
		cameraProperty.colorMode = COLOR_RAW8;
	}
	else if (sColorMode == "COLOR_RAW16")
	{
		cameraProperty.colorMode = COLOR_RAW16;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set video format, video mode and color mode" << std::endl;
	}


/// -----------------------------------------------------------------
/// Set resolution
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_RESOLUTION;
	std::string sImageWidth = "";
	std::string sImageHeight = "";
	int iImageWidth = -1;
	int iImageHeight = -1;

	m_ColorCameraParameters.m_ImageWidth.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_ImageWidth.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_ImageWidth >> sImageWidth;
	m_ColorCameraParameters.m_ImageHeight.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_ImageHeight.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_ImageHeight >> sImageHeight;

	if (sImageWidth == "AUTO" || sImageHeight == "Auto")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sImageWidth == "DEFAULT" || sImageHeight == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_ImageWidth.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_ImageWidth.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_ImageWidth >> iImageWidth;
		m_ColorCameraParameters.m_ImageHeight.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_ImageHeight.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_ImageHeight >> iImageHeight;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set resolution." << std::endl;
	}


/// -----------------------------------------------------------------
/// Set exposure time
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_EXPOSURE_TIME;
	std::string sExposureTime = "";

	m_ColorCameraParameters.m_ExposureTime.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_ExposureTime.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_ExposureTime >> sExposureTime;

	if (sExposureTime == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sExposureTime == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_ExposureTime.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_ExposureTime.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_ExposureTime >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set auto exposure." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set frame rate
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_FRAME_RATE;
	std::string sFrameRate = "";

	m_ColorCameraParameters.m_FrameRate.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_FrameRate.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_FrameRate >> sFrameRate;

	if (sFrameRate == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sFrameRate == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_FLOAT;
		m_ColorCameraParameters.m_FrameRate.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_FrameRate.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_FrameRate >> cameraProperty.floatData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set frame rate." << std::endl;
	}


/// -----------------------------------------------------------------
/// Set shutter
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_SHUTTER;
	std::string sShutter = "";

	m_ColorCameraParameters.m_Shutter.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_Shutter.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Shutter >> sShutter;

	if (sShutter == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sShutter == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_Shutter.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_Shutter.seekg(0);
		m_ColorCameraParameters.m_Shutter >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set shutter." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set brightness
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_BRIGHTNESS;
	std::string sBrightness = "";

	m_ColorCameraParameters.m_Brightness.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_Brightness.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Brightness >> sBrightness;

	if (sBrightness == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sBrightness == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_Brightness.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_Brightness.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_Brightness >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set brightness." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set gain
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_GAIN;
	std::string sGain = "";

	m_ColorCameraParameters.m_Gain.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_Gain.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Gain >> sGain;

	if (sGain == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sGain == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_Gain.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_Gain.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_Gain >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set gain." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set gamma
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_GAMMA;
	std::string sGamma = "";

	m_ColorCameraParameters.m_Gamma.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_Gamma.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Gamma >> sGamma;

	if (sGamma == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sGamma == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_Gamma.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_Gamma.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_Gamma >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set gamma." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set hue
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_HUE;
	std::string sHue = "";

	m_ColorCameraParameters.m_Hue.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_Hue.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Hue >> sHue;

	if (sHue == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sHue == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_Hue.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_Hue.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_Hue >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set hue." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set saturation
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_SATURATION;
	std::string sSaturation = "";

	m_ColorCameraParameters.m_Saturation.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_Saturation.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Saturation >> sSaturation;

	if (sSaturation == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sSaturation == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_Saturation.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_Saturation.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_Saturation >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set saturation." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set white balance U
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_WHITE_BALANCE_U;
	std::string sWhiteBalanceU = "";

	m_ColorCameraParameters.m_WhiteBalanceU.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_WhiteBalanceU.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_WhiteBalanceU >> sWhiteBalanceU;

	if (sWhiteBalanceU == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sWhiteBalanceU == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_WhiteBalanceU.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_WhiteBalanceU.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_WhiteBalanceU >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set white balance U." << std::endl;
	}

/// -----------------------------------------------------------------
/// Set white balance V
/// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_WHITE_BALANCE_V;
	std::string sWhiteBalanceV = "";

	m_ColorCameraParameters.m_WhiteBalanceV.clear(); /// Clear flags within stringstream
	m_ColorCameraParameters.m_WhiteBalanceV.seekg(0); /// Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_WhiteBalanceV >> sWhiteBalanceV;

	if (sWhiteBalanceV == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sWhiteBalanceV == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_ColorCameraParameters.m_WhiteBalanceV.clear(); /// Clear flags within stringstream
		m_ColorCameraParameters.m_WhiteBalanceV.seekg(0); /// Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_WhiteBalanceV >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - AVTPikeCam::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set white balance V." << std::endl;
	}
	return RET_OK;
}

unsigned long AVTPikeCam::LoadParameters(const char* filename, int cameraIndex)
{ 
	TiXmlDocument* p_configXmlDocument = new TiXmlDocument( filename );
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - AVTPikeCam::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << filename << "'" << std::endl;

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
//	BEGIN LibCameraSensors->AVTPikeCam
//************************************************************************************
			// Tag element "AVTPikeCam" of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_AVTPikeCam = NULL;
			std::stringstream ss;
			ss << "AVTPikeCam_" << cameraIndex;
			p_xmlElement_Root_AVTPikeCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_AVTPikeCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->GUID
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				std::string tempString;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "GUID" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "high", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'high' of tag 'GUID'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					m_GUID.High = strtoul(tempString.c_str(),NULL,16);

					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "low", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'low' of tag 'GUID'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					m_GUID.Low = strtoul(tempString.c_str(),NULL,16);
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ...  Can't find tag 'GUID'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->Role
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "Role" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'value' of tag 'Role'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_ColorCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_ColorCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Role'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->VideoFormat
//************************************************************************************
				// Subtag element "ImageFormat" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "VideoFormat" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "type", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'type' of tag 'VideoFormat'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_VideoFormat.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_VideoFormat.clear();		/// Reset flags
						m_ColorCameraParameters.m_VideoFormat << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'VideoFormat'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->VideoMode
//************************************************************************************
				// Subtag element "ColorFormat" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "VideoMode" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "type", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'type' of tag 'VideoMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_VideoMode.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_VideoMode.clear();		/// Reset flags
						m_ColorCameraParameters.m_VideoMode << tempString;
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'VideoMode'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->Resolution
//************************************************************************************
				// Subtag element "XSize" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "Resolution" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("width", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'width' of tag 'Resolution'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ImageWidth.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_ImageWidth.clear();		/// Reset flags
						m_ColorCameraParameters.m_ImageWidth << tempString;
					}
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("height", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'height' of tag 'Resolution'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ImageHeight.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_ImageHeight.clear();		/// Reset flags
						m_ColorCameraParameters.m_ImageHeight << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Resolution'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->FrameRate
//************************************************************************************
				// Subtag element "FrameRate" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "FrameRate" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("fps", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'fps' of tag 'FrameRate'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_FrameRate.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_FrameRate.clear();		/// Reset flags
						m_ColorCameraParameters.m_FrameRate << tempString;
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'FrameRate'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}


//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->ColorMode
//************************************************************************************
				// Subtag element "ColorFormat" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "ColorMode" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "type", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'type' of tag 'ColorMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ColorMode.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_ColorMode.clear();		/// Reset flags
						m_ColorCameraParameters.m_ColorMode << tempString;
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'ColorMode'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->IsoSpeed
//************************************************************************************
				// Subtag element "IsoSpeed " of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "IsoSpeed" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "speed", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'speed' of tag 'IsoSpeed'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_IsoSpeed.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_IsoSpeed.clear();		/// Reset flags
						m_ColorCameraParameters.m_IsoSpeed << tempString;
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'IsoSpeed'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->OperationMode
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "OperationMode" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'OperationMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "A") m_operationMode_B = false;
					else if (tempString == "B") m_operationMode_B = true;
					else
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... FireWire operation mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'OperationMode'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_SHUTTER
//************************************************************************************
				// Subtag element "PROP_SHUTTER" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_SHUTTER" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_SHUTTER'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_Shutter.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_Shutter.clear();		/// Reset flags
						m_ColorCameraParameters.m_Shutter << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_SHUTTER'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_BRIGHTNESS
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_BRIGHTNESS" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_BRIGHTNESS'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_Brightness.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_Brightness.clear();		/// Reset flags
						m_ColorCameraParameters.m_Brightness << tempString;
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_BRIGHTNES'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_EXPOSURE_TIME
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_EXPOSURE_TIME" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_EXPOSURE_TIME'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ExposureTime.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_ExposureTime.clear();		/// Reset flags
						m_ColorCameraParameters.m_ExposureTime << tempString;
					}

				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_EXPOSURE_TIME'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_WHITE_BALANCE_U 
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement("PROP_WHITE_BALANCE_U");
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_WHITE_BALANCE_U'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_WhiteBalanceU.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_WhiteBalanceU.clear();		/// Reset flags
						m_ColorCameraParameters.m_WhiteBalanceU << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_WHITE_BALANCE_U'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_WHITE_BALANCE_V 
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_WHITE_BALANCE_V" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_WHITE_BALANCE_V'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_WhiteBalanceV.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_WhiteBalanceV.clear();		/// Reset flags
						m_ColorCameraParameters.m_WhiteBalanceV << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_WHITE_BALANCE_V'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_HUE
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_HUE" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_HUE'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_Hue.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_Hue.clear();		/// Reset flags
						m_ColorCameraParameters.m_Hue << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_HUE'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_SATURATION
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_SATURATION" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_SATURATION'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_Saturation.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_Saturation.clear();		/// Reset flags
						m_ColorCameraParameters.m_Saturation << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_SATURATION'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_GAMMA 
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_GAMMA" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_GAMMA'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_Gamma.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_Gamma.clear();		/// Reset flags
						m_ColorCameraParameters.m_Gamma << tempString;
					}
				}
				else
				{
					std::cerr << "AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_GAMMA'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->PROP_GAIN 
//************************************************************************************
				// Subtag element "PROP_GAIN" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "PROP_GAIN" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'PROP_GAIN '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_Gain.str( " " );	/// Clear stringstream
						m_ColorCameraParameters.m_Gain.clear();		/// Reset flags
						m_ColorCameraParameters.m_Gain << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'PROP_EXPOSURE_TIME'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->AVTPikeCam
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - AVTPikeCam::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	return RET_OK;
}

