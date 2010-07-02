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
#include "cob_camera_sensors/VirtualColorCam.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/VirtualColorCam.h"
#endif

using namespace std;
using namespace ipa_CameraSensors;

#ifdef __cplusplus
extern "C" {
#endif
__DLL_ABSTRACTCOLORCAMERA_H__ AbstractColorCamera* APIENTRY CreateColorCamera_VirtualCam()

{
	return (new VirtualColorCam());
}
#ifdef __cplusplus
}
#endif



VirtualColorCam::VirtualColorCam()
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;

	m_ImageWidth = 0;
	m_ImageHeight = 0;

	m_ImageCounter = 0;
}

VirtualColorCam::~VirtualColorCam()
{
	if (isOpen())
	{
		Close();
	}
}


unsigned long VirtualColorCam::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_VIRTUALCOLOR;

	/// It is important to put this before LoadParameters
	m_CameraDataDirectory = directory;
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);
	}

	m_CameraIndex = cameraIndex;
		
	m_initialized = true;
	return RET_OK;

}


unsigned long VirtualColorCam::Open()
{
	if (!isInitialized())
	{
		std::cerr << "ERROR - VirtualColorCam::Open:" << std::endl;
		std::cerr << "\t ... Camera not initialized." << std::endl;
		return (RET_FAILED);
	}
	m_open = false;

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	/// Convert camera ID to string
	std::stringstream ss;
	std::string sCameraIndex;
	ss << m_CameraIndex;
	ss >> sCameraIndex;

	m_ImageWidth = -1;
	m_ImageHeight = -1;

	/// Create absolute filename and check if directory exists
	fs::path absoluteDirectoryName( m_CameraDataDirectory );
	if ( !fs::exists( absoluteDirectoryName ) )
	{
		std::cerr << "ERROR - VirtualColorCam::Open:" << std::endl;
		std::cerr << "\t ... Path '" << absoluteDirectoryName.file_string() << "' not found" << std::endl;
		return (ipa_CameraSensors::RET_FAILED | ipa_CameraSensors::RET_FAILED_OPEN_FILE);
	}

	int colorImageCounter = 0;
	/// Extract all image filenames from the directory
	if ( fs::is_directory( absoluteDirectoryName ) )
	{
		std::cout << "INFO - VirtualColorCam::Open:" << std::endl;
		std::cout << "\t ... Parsing directory '" << absoluteDirectoryName.directory_string() << "'" << std::endl;;
	    fs::directory_iterator end_iter;
		for ( fs::directory_iterator dir_itr( absoluteDirectoryName ); dir_itr != end_iter; ++dir_itr )
		{
			try
			{
				if (fs::is_regular_file(dir_itr->status()))
				{
					std::string filename = dir_itr->path().string();
					if ((dir_itr->path().extension() == ".jpg" || dir_itr->path().extension() == ".jpe" ||
						dir_itr->path().extension() == ".jpeg" || dir_itr->path().extension() == ".bmp" ||
						dir_itr->path().extension() == ".bmp" || dir_itr->path().extension() == ".dib" ||
						dir_itr->path().extension() == ".png" || dir_itr->path().extension() == ".pgm" ||
						dir_itr->path().extension() == ".ppm" || dir_itr->path().extension() == ".sr" ||
						dir_itr->path().extension() == ".ras" || dir_itr->path().extension() == ".tiff" ||
						dir_itr->path().extension() == ".exr" || dir_itr->path().extension() == ".jp2") &&
						filename.find( "ColorCamRGB_8U3_" + sCameraIndex, 0 ) != std::string::npos)
					{
						++colorImageCounter;
						//std::cout << "VirtualColorCam::Open(): Reading '" << dir_itr->path().string() << "\n";
						m_ColorImageFileNames.push_back(dir_itr->path().string());
						/// Get image size
						if (m_ImageWidth == -1 || m_ImageHeight == -1)
						{
							IplImage* image = (IplImage*) cvLoadImage(m_ColorImageFileNames.back().c_str(), CV_LOAD_IMAGE_COLOR);
							m_ImageWidth = image->width;
							m_ImageHeight = image->height;
							cvReleaseImage(&image);
						}
					}
				}
			}
			catch ( const std::exception &ex )
			{
				std::cerr << "ERROR - VirtualColorCam::Open:" << std::endl;
				std::cerr << "\t ... Exception catch of '" << ex.what() << "'" << std::endl;
			}
		}
		std::sort(m_ColorImageFileNames.begin(),m_ColorImageFileNames.end());
		std::cout << "INFO - VirtualColorCam::Open:" << std::endl;
		std::cerr << "\t ... Extracted '" << colorImageCounter << "' color images (8*3 bit/color)\n";
	}
	else
	{
		std::cerr << "ERROR - VirtualColorCam::Open:" << std::endl;
		std::cerr << "\t .... Path '" << absoluteDirectoryName.file_string() << "' is not a directory." << std::endl;
		return ipa_CameraSensors::RET_FAILED;
	}

	if (colorImageCounter == 0)
	{
		std::cerr << "ERROR - VirtualColorCam::Open:" << std::endl;
		std::cerr << "\t ... Could not detect any color images" << std::endl;
		std::cerr << "\t ... from the specified directory. Check directory" << std::endl;
		std::cerr << "\t ... and filenames (i.e. ColorCamRGB_8U3_*_*.jpg)." << std::endl;
		return ipa_CameraSensors::RET_FAILED;
	}

	std::cout << "*******************************************************" << std::endl;
	std::cout << "VirtualColorCam::Open: Virtual color camera device OPEN" << std::endl;
	std::cout << "*******************************************************" << std::endl << std::endl;

	m_open = true;
	return RET_OK;

}

int VirtualColorCam::GetNumberOfImages()
{
	return (int)std::min(0.0f, (float)m_ColorImageFileNames.size());
}

unsigned long VirtualColorCam::SaveParameters(const char* filename)
{ 
	return RET_FAILED;
}

unsigned long VirtualColorCam::SetPathToImages(std::string path) 
{
	m_CameraDataDirectory = path;
	return RET_OK;
}


unsigned long VirtualColorCam::Close()
{
	if (!isOpen())
	{
		return RET_OK;
	}

	m_open = false;
	return RET_OK;
} 


unsigned long VirtualColorCam::SetProperty(t_cameraProperty* cameraProperty)
{
#ifdef __LINUX__
	std::cerr << "VirtualColorCam::SetProperty: Function not implemented.";
	return RET_FAILED;
#endif
#ifndef __LINUX__

	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:
			m_ImageWidth = cameraProperty->cameraResolution.xResolution;
			m_ImageHeight = cameraProperty->cameraResolution.yResolution;
			break;
		default: 				
			std::cerr << "ERROR - VirtualColorCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return RET_FAILED;
			break;
	}

	return RET_OK;
#endif
}

unsigned long VirtualColorCam::SetPropertyDefaults() {return RET_FUNCTION_NOT_IMPLEMENTED;}

unsigned long VirtualColorCam::GetProperty(t_cameraProperty* cameraProperty)
{
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:	
			cameraProperty->cameraResolution.xResolution = m_ImageWidth;
			cameraProperty->cameraResolution.yResolution = m_ImageHeight;
			cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			return RET_OK;
			break;

		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;

		default: 				
			std::cerr << "ERROR - VirtualColorCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return RET_FAILED;
			break;

	}

	return RET_OK;
} 


unsigned long VirtualColorCam::GetColorImage(char* colorImageData, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - VirtualColorCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}
	
	IplImage* colorImage = (IplImage*) cvLoadImage(m_ColorImageFileNames[m_ImageCounter].c_str(), CV_LOAD_IMAGE_COLOR);

	for(int row=0; row<m_ImageHeight; row++)
	{
		for (int col=0; col<m_ImageWidth; col++)
		{
			char* f_color_ptr = &((char*) (colorImage->imageData + row*colorImage->widthStep))[col*3];
			((char*) (colorImageData + row*colorImage->widthStep))[col*3 + 0] = f_color_ptr[0];
			((char*) (colorImageData + row*colorImage->widthStep))[col*3 + 1] = f_color_ptr[1];
			((char*) (colorImageData + row*colorImage->widthStep))[col*3 + 2] = f_color_ptr[2];
		}	
	}
	
	cvReleaseImage(&colorImage);

	m_ImageCounter++;
	if (m_ImageCounter >= m_ColorImageFileNames.size())
	{
		/// Reset image counter
		m_ImageCounter = 0;
	}

	return RET_OK;
}


unsigned long VirtualColorCam::GetColorImage(IplImage* colorImage, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - VirtualColorCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	if(colorImage->depth == IPL_DEPTH_8U &&
		colorImage->nChannels == 3 &&
		colorImage->width == (int) m_ImageWidth &&
		colorImage->height == (int) m_ImageHeight)
	{
		return GetColorImage(colorImage->imageData, getLatestFrame);
	}
	else
	{
		std::cerr << "ERROR - VirtualColorCam::GetColorImage:" << std::endl;
		std::cerr << "\t ... Could not acquire color image. IplImage initialized with wrong attributes." << std::endl;
		return RET_FAILED;
	}

	return RET_FAILED;
}

unsigned long VirtualColorCam::GetColorImage2(IplImage** colorImage, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - VirtualColorCam::GetColorImage2:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	*colorImage = cvCreateImage(cvSize(m_ImageWidth, m_ImageHeight), IPL_DEPTH_8U, 3);
	return GetColorImage((*colorImage)->imageData, getLatestFrame);
	
	return RET_FAILED;
}


unsigned long VirtualColorCam::PrintCameraInformation()
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long VirtualColorCam::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}

	return RET_OK;
}


unsigned long VirtualColorCam::LoadParameters(const char* filename, int cameraIndex)
{  //m_intrinsicMatrix; m_distortionParameters
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - VirtualColorCam::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n" << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - VirtualColorCam::LoadParameters:" << std::endl;
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
//	BEGIN LibCameraSensors->VirtualColorCam
//************************************************************************************
			// Tag element "VirtualColorCam" of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_VirtualColorCam = NULL;
			std::stringstream ss;
			ss << "VirtualColorCam_" << cameraIndex;
			p_xmlElement_Root_VirtualColorCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_VirtualColorCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->VirtualColorCam->CameraDataDirectory
//************************************************************************************
				// Subtag element "CameraDataDirectory" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_VirtualColorCam->FirstChildElement( "CameraDataDirectory" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					std::string tempString;
					if ( p_xmlElement_Child->QueryValueAttribute( "relativePath", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "VirtualColorCam::LoadParameters: Can't find attribute 'relativePath' of tag 'CameraDataDirectory'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					
					m_CameraDataDirectory = m_CameraDataDirectory + tempString + "/";
				}
				else
				{
					std::cerr << "ERROR - VirtualColorCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CameraDataDirectory'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}
//************************************************************************************
//	END LibCameraSensors->VirtualColorCam
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - VirtualColorCam::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - VirtualColorCam::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	std::cout << "\t [OK] Parsing xml configuration file           " << std::endl;

	return RET_OK;
}
