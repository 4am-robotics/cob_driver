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

#ifdef __LINUX__
#include "cob_camera_sensors/VirtualRangeCam.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/VirtualRangeCam.h"
#endif

using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_VirtualCam()
{
	return AbstractRangeImagingSensorPtr(new VirtualRangeCam());
}

VirtualRangeCam::VirtualRangeCam()
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;

	m_ImageCounter = 0;
}


VirtualRangeCam::~VirtualRangeCam()
{
	if (isOpen())
	{
		Close();
	}
}

unsigned long VirtualRangeCam::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_VIRTUALRANGE;

	// It is important to put this before LoadParameters
	m_CameraDataDirectory = directory;

	// Load SR parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - VirtualRangeCam::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed" << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);
	}

	m_CoeffsInitialized = true;
	if (m_CalibrationMethod == MATLAB)
	{
		// Load z-calibration files
		std::string filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA0.xml";
		CvMat* c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA0.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA0 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA1.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA1.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA1 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA2.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA2.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA2 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA3.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA3.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA3 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA4.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA4.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA4 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA5.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA5.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA5 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA6.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA6.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA6 = c_mat;
			cvReleaseMat(&c_mat);
		}
	}

	m_CameraIndex = cameraIndex;

	// set init flag
	m_initialized = true;

	return RET_OK;
}


inline void VirtualRangeCam::UpdateImageDimensionsOnFirstImage(std::string filename, std::string ext)
{
	if (m_ImageWidth == -1 || m_ImageHeight == -1)
	{
		if (ext != ".bin")
		{
			IplImage* image = (IplImage*) cvLoad(filename.c_str(), 0);
			m_ImageWidth = image->width;
			m_ImageHeight = image->height;
			cvReleaseImage(&image);
		}
		else
		{
			cv::Mat mat;
			ipa_Utils::LoadMat(mat, filename);
			m_ImageHeight = mat.rows;
			m_ImageWidth = mat.cols;
		}
	}
}


inline void VirtualRangeCam::FindSourceImageFormat(std::map<std::string, int>::iterator& itCounter, std::string& ext)
{
	if (itCounter->second > 0)
	{
		if (ext == "") ext = itCounter->first;
		else
		{
			std::cerr << "ERROR - VirtualRangeCam::Open:\n";
			std::cerr << "\t ... The provided path contains data in mixed formats (e.g. .xml and .bin).\n";
			assert(false);
		}
	}
}


unsigned long VirtualRangeCam::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	// Convert camera ID to string
	std::stringstream ss;
	std::string sCameraIndex;
	ss << m_CameraIndex;
	ss >> sCameraIndex;

	m_ImageWidth = -1;
	m_ImageHeight = -1;

	// Create absolute filename and check if directory exists
	fs::path absoluteDirectoryName( m_CameraDataDirectory );
	if ( !fs::exists( absoluteDirectoryName ) )
	{
		std::cerr << "ERROR - VirtualRangeCam::Open:" << std::endl;
		std::cerr << "\t ... Path '" << absoluteDirectoryName.file_string() << "' not found" << std::endl;
		return (ipa_CameraSensors::RET_FAILED | ipa_CameraSensors::RET_FAILED_OPEN_FILE);
	}

	std::vector<std::string> extensionList;
	extensionList.push_back(".xml"); extensionList.push_back(".bin"); extensionList.push_back(".png"); extensionList.push_back(".jpg"); extensionList.push_back(".bmp");
	std::map<std::string, int> amplitudeImageCounter;	// first index is the extension (.xml, .bin), second is the number of such images found
	std::map<std::string, int> intensityImageCounter;	// first index is the extension (.xml, .bin, .png, .jpg, .bmp), second is the number of such images found
	std::map<std::string, int> coordinateImageCounter;	// first index is the extension (.xml, .bin), second is the number of such images found
	std::map<std::string, int> rangeImageCounter;		// first index is the extension (.xml, .bin), second is the number of such images found
	std::map<std::string, std::vector<std::string> > amplitudeImageFileNames;	// first index is the extension (.xml, .bin), second is the vector of file names
	std::map<std::string, std::vector<std::string> > intensityImageFileNames;	// first index is the extension (.xml, .bin, .png, .jpg, .bmp), second is the vector of file names
	std::map<std::string, std::vector<std::string> > coordinateImageFileNames;	// first index is the extension (.xml, .bin), second is the vector of file names
	std::map<std::string, std::vector<std::string> > rangeImageFileNames;		// first index is the extension (.xml, .bin), second is the vector of file names
	// Extract all image filenames from the directory
	if ( fs::exists( absoluteDirectoryName ) )
	{
		std::cout << "INFO - VirtualRangeCam::Open   :" << std::endl;
		std::cout << "\t ... Parsing directory '" << absoluteDirectoryName.directory_string() << "'" << std::endl;
		
		fs::directory_iterator end_iter;
		for ( fs::directory_iterator dir_itr( absoluteDirectoryName ); dir_itr != end_iter; ++dir_itr )
		{
			try
			{
				if (fs::is_regular_file(dir_itr->status()))
				{
					std::string filename = dir_itr->path().string();

					// intensity image formats
					for (unsigned int extIndex=0; extIndex<extensionList.size(); extIndex++)
					{
						std::string ext = extensionList[extIndex];
						std::string format = "32F1_";
						if (extIndex>=2) format = "8U3_";
						if ((dir_itr->path().extension() == ext) && filename.find( "RangeCamIntensity_" + format + sCameraIndex, 0 ) != std::string::npos)
						{
							(intensityImageCounter.find(ext) == intensityImageCounter.end()) ? intensityImageCounter[ext] = 1 : intensityImageCounter[ext]++;
							//std::cout << "VirtualRangeCam::Open(): Reading '" << filename << "\n";
							intensityImageFileNames[ext].push_back(filename);
							UpdateImageDimensionsOnFirstImage(filename, ext);
						}
					}

					// amplitude image formats
					for (unsigned int extIndex=0; extIndex<2; extIndex++)
					{
						std::string ext = extensionList[extIndex];
						if ((dir_itr->path().extension() == ext) && filename.find( "RangeCamAmplitude_32F1_" + sCameraIndex, 0 ) != std::string::npos)
						{
							(amplitudeImageCounter.find(ext) == amplitudeImageCounter.end()) ? amplitudeImageCounter[ext] = 1 : amplitudeImageCounter[ext]++;
							//std::cout << "VirtualRangeCam::Open(): Reading '" << filename << "\n";
							amplitudeImageFileNames[ext].push_back(filename);
							UpdateImageDimensionsOnFirstImage(filename, ext);
						}
					}

					// coordinate image formats
					for (unsigned int extIndex=0; extIndex<2; extIndex++)
					{
						std::string ext = extensionList[extIndex];
						if ((dir_itr->path().extension() == ext) && filename.find( "RangeCamCoordinate_32F3_" + sCameraIndex, 0 ) != std::string::npos)
						{
							(coordinateImageCounter.find(ext) == coordinateImageCounter.end()) ? coordinateImageCounter[ext] = 1 : coordinateImageCounter[ext]++;
							coordinateImageFileNames[ext].push_back(filename);
							UpdateImageDimensionsOnFirstImage(filename, ext);
							//std::cout << "VirtualRangeCam::Open(): Reading '" << filename << "\n";
						}
					}

					// range image formats
					for (unsigned int extIndex=0; extIndex<2; extIndex++)
					{
						std::string ext = extensionList[extIndex];
						if ((dir_itr->path().extension() == ext) && filename.find( "RangeCamRange_32F1_" + sCameraIndex, 0 ) != std::string::npos)
						{
							(rangeImageCounter.find(ext) == rangeImageCounter.end()) ? rangeImageCounter[ext] = 1 : rangeImageCounter[ext]++;
							//std::cout << "VirtualRangeCam::Open(): Reading '" << filename << "\n";
							rangeImageFileNames[ext].push_back(filename);
							UpdateImageDimensionsOnFirstImage(filename, ext);
						}
					}
				}
			}
			catch ( const std::exception &ex )
			{
				std::cout << "WARNING - VirtualRangeCam::Open:" << std::endl;
				std::cout << "\t ... Exception catch of '" << ex.what() << "'" << std::endl;
			}
		}
		// intensity
		std::map<std::string, int>::iterator itCounter;
		std::string extInt = "";
		for (itCounter = intensityImageCounter.begin(); itCounter != intensityImageCounter.end(); itCounter++) FindSourceImageFormat(itCounter, extInt);
		if (extInt != "") m_IntensityImageFileNames = intensityImageFileNames[extInt];

		// amplitude
		std::string extAmp = "";
		for (itCounter = amplitudeImageCounter.begin(); itCounter != amplitudeImageCounter.end(); itCounter++) FindSourceImageFormat(itCounter, extAmp);
		if (extAmp != "") m_AmplitudeImageFileNames = amplitudeImageFileNames[extAmp];

		// coordinates
		std::string extCoord = "";
		for (itCounter = coordinateImageCounter.begin(); itCounter != coordinateImageCounter.end(); itCounter++) FindSourceImageFormat(itCounter, extCoord);
		if (extCoord != "") m_CoordinateImageFileNames = coordinateImageFileNames[extCoord];

		// range
		std::string extRange = "";
		for (itCounter = rangeImageCounter.begin(); itCounter != rangeImageCounter.end(); itCounter++) FindSourceImageFormat(itCounter, extRange);
		if (extRange != "") m_RangeImageFileNames = rangeImageFileNames[extRange];

		std::sort(m_IntensityImageFileNames.begin(),m_IntensityImageFileNames.end());
		std::sort(m_AmplitudeImageFileNames.begin(),m_AmplitudeImageFileNames.end());
		std::sort(m_CoordinateImageFileNames.begin(),m_CoordinateImageFileNames.end());
		std::sort(m_RangeImageFileNames.begin(),m_RangeImageFileNames.end());
		std::cout << "INFO - VirtualRangeCam::Open:" << std::endl;
		std::cout << "\t ... Extracted '" << intensityImageCounter[extInt] << "' intensity images (3*8 or 16 bit/value)\n";
		std::cout << "\t ... Extracted '" << amplitudeImageCounter[extAmp] << "' amplitude images (16 bit/value)\n";
		std::cout << "\t ... Extracted '" << coordinateImageCounter[extCoord] << "' coordinate images (3*16 bit/value)\n";
		std::cout << "\t ... Extracted '" << rangeImageCounter[extRange] << "' range images (16 bit/value)\n";

		if (intensityImageCounter[extInt] == 0 && amplitudeImageCounter[extAmp] == 0)
		{
			std::cerr << "ERROR - VirtualRangeCam::Open:" << std::endl;
			std::cerr << "\t ... Could not detect any intensity or amplitude images" << std::endl;
			std::cerr << "\t ... from the specified directory." << std::endl;
			return ipa_CameraSensors::RET_FAILED;
		}

		if (coordinateImageCounter[extCoord] != 0 &&
			((intensityImageCounter[extInt] != coordinateImageCounter[extCoord] &&
			amplitudeImageCounter[extAmp] != coordinateImageCounter[extCoord]) ||
			(coordinateImageCounter[extCoord] != rangeImageCounter[extRange])))
		{
			std::cerr << "ERROR - VirtualRangeCam::Open:" << std::endl;
			std::cerr << "\t ... Number of intensity, range and coordinate images must agree." << std::endl;
			return ipa_CameraSensors::RET_FAILED;
		}

		if((m_CalibrationMethod == NATIVE || m_CalibrationMethod == MATLAB_NO_Z) && coordinateImageCounter[extCoord] == 0 )
		{
			std::cerr << "ERROR - VirtualRangeCam::Open:" << std::endl;
			std::cerr << "\t ... Coordinate images must be available for calibration mode NATIVE or MATLAB_NO_Z." << std::endl;
			return ipa_CameraSensors::RET_FAILED;
		}
	}
	else
	{
		std::cerr << "ERROR - VirtualRangeCam::Open():" << std::endl;
		std::cerr << "\t ... Path '" << absoluteDirectoryName.file_string() << "' is not a directory." << std::endl;
		return ipa_CameraSensors::RET_FAILED;
	}


	std::cout << "**************************************************" << std::endl;
	std::cout << "VirtualRangeCam::Open(): Virtual range camera OPEN" << std::endl;
	std::cout << "**************************************************" << std::endl<< std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long VirtualRangeCam::Close()
{
	if (!isOpen())
	{
		return (RET_OK);
	}

	m_open = false;
	return RET_OK;

}


unsigned long VirtualRangeCam::SetProperty(t_cameraProperty* cameraProperty)
{
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:

		default:
			std::cerr << "ERROR - VirtualRangeCam::SetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified.\n";
			return RET_FAILED;
			break;
	}

	return RET_OK;
}


unsigned long VirtualRangeCam::SetPropertyDefaults()
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}


unsigned long VirtualRangeCam::GetProperty(t_cameraProperty* cameraProperty)
{
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:
			cameraProperty->cameraResolution.xResolution = m_ImageWidth;
			cameraProperty->cameraResolution.yResolution = m_ImageHeight;
			cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			break;

		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;

		default:
			std::cerr << "VirtualRangeCam::GetProperty:" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return RET_FAILED;
			break;

	}

	return RET_OK;
}


// Wrapper for IplImage retrival from AcquireImage
// Images have to be initialized prior to calling this function
unsigned long VirtualRangeCam::AcquireImages(cv::Mat* rangeImage, cv::Mat* grayImage, cv::Mat* cartesianImage,
											 bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	//std::cout << m_ImageCounter << std::endl;

	char* rangeImageData = 0;
	char* grayImageData = 0;
	char* cartesianImageData = 0;
	int widthStepRange = -1;
	int widthStepGray = -1;
	int widthStepCartesian = -1;

	if(rangeImage)
	{
		rangeImage->create(m_ImageHeight, m_ImageWidth, CV_32FC1);
		rangeImageData = (char*) rangeImage->data;
		widthStepRange = rangeImage->step;
	}

	if(grayImage)
	{
		if (grayImageType == ipa_CameraSensors::INTENSITY_8U3) grayImage->create(m_ImageHeight, m_ImageWidth, CV_8UC3);
		else grayImage->create(m_ImageHeight, m_ImageWidth, CV_32FC1);
		grayImageData = (char*) grayImage->data;
		widthStepGray = grayImage->step;
	}

	if(cartesianImage)
	{
		cartesianImage->create(m_ImageHeight, m_ImageWidth, CV_32FC3);
		cartesianImageData = (char*) cartesianImage->data;
		widthStepCartesian = cartesianImage->step;
	}

	if (widthStepRange+widthStepGray+widthStepCartesian == -3)
	{
		return RET_OK;
	}

	return AcquireImages(widthStepRange, widthStepGray, widthStepCartesian, rangeImageData, grayImageData, cartesianImageData, getLatestFrame, undistort, grayImageType);

}

unsigned long VirtualRangeCam::AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImageData, char* grayImageData, char* cartesianImageData,
											 bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	if (!m_open)
	{
		std::cerr << "ERROR - VirtualRangeCam::AcquireImages:" << std::endl;
		std::cerr << "\t ... Camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

///***********************************************************************
// Range image (distorted or undistorted)
///***********************************************************************
	if (rangeImageData)
	{
		float* f_ptr = 0;
		float* f_ptr_dst = 0;
		bool releaseNecessary = false;

		cv::Mat rangeMat;
		IplImage rangeIpl;
		IplImage* rangeImage = &rangeIpl;
		if (m_RangeImageFileNames[m_ImageCounter].find(".bin") != std::string::npos)
		{
			ipa_Utils::LoadMat(rangeMat, m_RangeImageFileNames[m_ImageCounter]);
			rangeIpl = (IplImage)rangeMat;
		}
		else if (m_RangeImageFileNames[m_ImageCounter].find(".xml") != std::string::npos)
		{
			rangeImage = (IplImage*) cvLoad(m_RangeImageFileNames[m_ImageCounter].c_str(), 0);
			releaseNecessary = true;
		}
		else
		{
			std::cerr << "ERROR - VirtualRangeCam::AcquireImages:\n";
			std::cerr << "\t ... Wrong file format for file " << m_RangeImageFileNames[m_ImageCounter] << ".\n";
			CV_Assert(false);
		}
		
		if (!undistort)
		{
			// put data in corresponding IPLImage structures
			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (rangeImage->imageData + row*rangeImage->widthStep);
				f_ptr_dst = (float*) (rangeImageData + row*widthStepRange);

				for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
				{
					f_ptr_dst[col] = f_ptr[col];
				}
			}
		}
		else
		{
			cv::Mat undistortedData (m_ImageHeight, m_ImageWidth, CV_32FC1, (float*) rangeImageData);
			cv::Mat cpp_rangeImage = rangeImage;

			assert (!m_undistortMapX.empty() && !m_undistortMapY.empty());
			cv::remap(cpp_rangeImage, undistortedData, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);
		}

		if (releaseNecessary) cvReleaseImage(&rangeImage);
	} // End if (rangeImage)
///***********************************************************************
// Gray image based on amplitude or intensity (distorted or undistorted)
///***********************************************************************
	if(grayImageData)
	{
		float* f_ptr = 0;
		float* f_ptr_dst = 0;
		unsigned char* uc_ptr = 0;
		unsigned char* uc_ptr_dst = 0;
		cv::Mat grayMat;
		IplImage grayIpl;
		IplImage* grayImage = &grayIpl;
		bool releaseNecessary = false;

		// load image
		if ((grayImageType == ipa_CameraSensors::INTENSITY_32F1) || (grayImageType == ipa_CameraSensors::INTENSITY_8U3))
		{
			// intensity image
			if (m_IntensityImageFileNames[m_ImageCounter].find(".bin") != std::string::npos)
			{
				ipa_Utils::LoadMat(grayMat, m_IntensityImageFileNames[m_ImageCounter]);
				grayIpl = (IplImage)grayMat;
			}
			else if (m_IntensityImageFileNames[m_ImageCounter].find(".xml") != std::string::npos)
			{
				grayImage = (IplImage*) cvLoad(m_IntensityImageFileNames[m_ImageCounter].c_str());
				releaseNecessary = true;
			}
			else if ((m_IntensityImageFileNames[m_ImageCounter].find(".png") != std::string::npos) || (m_IntensityImageFileNames[m_ImageCounter].find(".bmp") != std::string::npos) ||
					(m_IntensityImageFileNames[m_ImageCounter].find(".jpg") != std::string::npos))
			{
				grayMat = cv::imread(m_IntensityImageFileNames[m_ImageCounter], -1);
				grayIpl = (IplImage)grayMat;
			}
			else
			{
				std::cerr << "ERROR - VirtualRangeCam::AcquireImages:\n";
				std::cerr << "\t ... Wrong file format for file " << m_IntensityImageFileNames[m_ImageCounter] << ".\n";
				CV_Assert(false);
			}
		}
		else
		{
			// amplitude image
			if (m_AmplitudeImageFileNames[m_ImageCounter].find(".bin") != std::string::npos)
			{
				ipa_Utils::LoadMat(grayMat, m_AmplitudeImageFileNames[m_ImageCounter]);
				grayIpl = (IplImage)grayMat;
			}
			else if (m_AmplitudeImageFileNames[m_ImageCounter].find(".xml") != std::string::npos)
			{
				grayImage = (IplImage*) cvLoad(m_AmplitudeImageFileNames[m_ImageCounter].c_str());
				releaseNecessary = true;
			}
			else
			{
				std::cerr << "ERROR - VirtualRangeCam::AcquireImages:\n";
				std::cerr << "\t ... Wrong file format for file " << m_AmplitudeImageFileNames[m_ImageCounter] << ".\n";
				CV_Assert(false);
			}
		}
		
		// process image
		if (!undistort)
		{
			if (grayImageType == ipa_CameraSensors::INTENSITY_8U3)
			{
				for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
				{
					f_ptr = (float*) (grayImage->imageData + row*grayImage->widthStep);
					f_ptr_dst = (float*) (grayImageData + row*widthStepGray);

					for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
					{
						f_ptr_dst[col] = f_ptr[col];
					}
				}
			}
			else
			{
				for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
				{
					uc_ptr = (unsigned char*) (grayImage->imageData + row*grayImage->widthStep);
					uc_ptr_dst = (unsigned char*) (grayImageData + row*widthStepGray);

					for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
					{
						uc_ptr_dst[col] = uc_ptr[col];
					}
				}
			}
		}
		else
		{
			cv::Mat undistortedData;
			if (grayImageType == ipa_CameraSensors::INTENSITY_8U3)
			{
				undistortedData = cv::Mat(m_ImageHeight, m_ImageWidth, CV_8UC3, (unsigned char*) grayImageData);
			}
			else
			{
				undistortedData = cv::Mat(m_ImageHeight, m_ImageWidth, CV_32FC1, (float*) grayImageData);
			}
			cv::Mat cpp_grayImage = grayImage;

			assert (!m_undistortMapX.empty() && !m_undistortMapY.empty());
			cv::remap(cpp_grayImage, undistortedData, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);
		}

		if (releaseNecessary) cvReleaseImage(&grayImage);
	}
///***********************************************************************
// Cartesian image (always undistorted)
///***********************************************************************
	if(cartesianImageData)
	{
		float x = -1;
		float y = -1;
		float zCalibrated = -1;
		float* f_ptr = 0;
		float* f_ptr_dst = 0;
		bool releaseNecessary = false;

		if(m_CalibrationMethod==MATLAB)
		{
			CV_Assert(false);
		}
		else if(m_CalibrationMethod==MATLAB_NO_Z)
		{
			// XYZ image is assumed to be undistorted
			// Unfortunately we have no access to the swissranger calibration

			cv::Mat coordinateMat;
			IplImage coordinateIpl;
			IplImage* coordinateImage = &coordinateIpl;
			if (m_CoordinateImageFileNames[m_ImageCounter].find(".bin") != std::string::npos)
			{
				ipa_Utils::LoadMat(coordinateMat, m_CoordinateImageFileNames[m_ImageCounter]);
				coordinateIpl = (IplImage)coordinateMat;
			}
			else if (m_CoordinateImageFileNames[m_ImageCounter].find(".xml") != std::string::npos)
			{
				coordinateImage = (IplImage*) cvLoad(m_CoordinateImageFileNames[m_ImageCounter].c_str(), 0);
				releaseNecessary = true;
			}
			else
			{
				std::cerr << "ERROR - VirtualRangeCam::AcquireImages:\n";
				std::cerr << "\t ... Wrong file format for file " << m_CoordinateImageFileNames[m_ImageCounter] << ".\n";
				CV_Assert(false);
			}

			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (coordinateImage->imageData + row*coordinateImage->widthStep);
				f_ptr_dst = (float*) (cartesianImageData + row*widthStepCartesian);

				for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
				{
					int colTimes3 = 3*col;

					zCalibrated = f_ptr[colTimes3+2];
					GetCalibratedXYMatlab(col, row, zCalibrated, x, y);

					f_ptr_dst[colTimes3] = x;
					f_ptr_dst[colTimes3 + 1] = y;
					f_ptr_dst[colTimes3 + 2] = zCalibrated;

					if (f_ptr_dst[colTimes3 + 2] < 0)
					{
						std::cout << "<0: " << row << " " << col << "\n";
					}
				}
			}
			if (releaseNecessary) cvReleaseImage(&coordinateImage);
		}
		else if(m_CalibrationMethod==NATIVE)
		{
			cv::Mat coordinateMat;
			IplImage coordinateIpl;
			IplImage* coordinateImage = &coordinateIpl;
			if (m_CoordinateImageFileNames[m_ImageCounter].find(".bin") != std::string::npos)
			{
				ipa_Utils::LoadMat(coordinateMat, m_CoordinateImageFileNames[m_ImageCounter]);
				coordinateIpl = (IplImage)coordinateMat;
			}
			else if (m_CoordinateImageFileNames[m_ImageCounter].find(".xml") != std::string::npos)
			{
				coordinateImage = (IplImage*) cvLoad(m_CoordinateImageFileNames[m_ImageCounter].c_str(), 0);
				releaseNecessary = true;
			}
			else
			{
				std::cerr << "ERROR - VirtualRangeCam::AcquireImages:\n";
				std::cerr << "\t ... Wrong file format for file " << m_CoordinateImageFileNames[m_ImageCounter] << ".\n";
				CV_Assert(false);
			}

			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (coordinateImage->imageData + row*coordinateImage->widthStep);
				f_ptr_dst = (float*) (cartesianImageData + row*widthStepCartesian);

				for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
				{
					int colTimes3 = 3*col;

					f_ptr_dst[colTimes3] =f_ptr[colTimes3+0];;
					f_ptr_dst[colTimes3 + 1] = f_ptr[colTimes3+1];
					f_ptr_dst[colTimes3 + 2] = f_ptr[colTimes3+2];
				}
			}
			if (releaseNecessary) cvReleaseImage(&coordinateImage);
		}
		else
		{
			std::cerr << "ERROR - VirtualRangeCam::AcquireImages:" << std::endl;
			std::cerr << "\t ... Calibration method unknown.\n";
			return RET_FAILED;
		}
	}

	std::cout << m_ImageCounter << "        \r" << std::endl;
	m_ImageCounter++;
	if ((m_IntensityImageFileNames.size() != 0 && m_ImageCounter >= m_IntensityImageFileNames.size()) ||
		(m_AmplitudeImageFileNames.size() != 0 && m_ImageCounter >= m_AmplitudeImageFileNames.size()) ||
		(m_RangeImageFileNames.size() != 0 && m_ImageCounter >= m_RangeImageFileNames.size()) ||
		(m_CoordinateImageFileNames.size() != 0 && m_ImageCounter >= m_CoordinateImageFileNames.size()))
	{
		// Reset image counter
		m_ImageCounter = 0;
	}

	return RET_OK;
}

int VirtualRangeCam::GetNumberOfImages()
{
	if (m_IntensityImageFileNames.size() == 0 && 
		m_AmplitudeImageFileNames.size() == 0 &&
		m_RangeImageFileNames.size() == 0 && 
		m_CoordinateImageFileNames.size() == 0)
	{
		return 0;
	}

	int min=std::numeric_limits<int>::max();

	if (m_IntensityImageFileNames.size() != 0) min = (int)std::min((float)min, (float)m_IntensityImageFileNames.size());
	if (m_AmplitudeImageFileNames.size() != 0) min = (int)std::min((float)min, (float)m_AmplitudeImageFileNames.size());
	if (m_RangeImageFileNames.size() != 0) min = (int)std::min((float)min, (float)m_RangeImageFileNames.size());
	if (m_CoordinateImageFileNames.size() != 0) min = (int)std::min((float)min, (float)m_CoordinateImageFileNames.size());

	return min;
}

unsigned long VirtualRangeCam::SetPathToImages(std::string path) 
{
	m_CameraDataDirectory = path;
	return ipa_Utils::RET_OK;
}

unsigned long VirtualRangeCam::SaveParameters(const char* filename)
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}


// Assert, that <code>m_CoeffsA.size()</code> and <code>m_CoeffsB.size()</code> is initialized
// Before calling <code>GetCalibratedZMatlab</code>
unsigned long VirtualRangeCam::GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated)
{
	double c[7] = {m_CoeffsA0.at<double>(v,u), m_CoeffsA1.at<double>(v,u), m_CoeffsA2.at<double>(v,u), 
		m_CoeffsA3.at<double>(v,u), m_CoeffsA4.at<double>(v,u), m_CoeffsA5.at<double>(v,u), m_CoeffsA6.at<double>(v,u)};
	double y = 0;

	ipa_Utils::EvaluatePolynomial((double) zRaw, 6, &c[0], &y);

	zCalibrated = (float) y;
	return RET_OK;
}

unsigned long VirtualRangeCam::GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y)
{
	// Conversion form m to mm
	z *= 1000;

	// Use intrinsic camera parameters
	double fx, fy, cx, cy;

	fx = m_intrinsicMatrix.at<double>(0, 0);
	fy = m_intrinsicMatrix.at<double>(1, 1);

	cx = m_intrinsicMatrix.at<double>(0, 2);
	cy = m_intrinsicMatrix.at<double>(1, 2);

	// Fundamental equation: u = (fx*x)/z + cx
	if (fx == 0)
	{
		std::cerr << "VirtualRangeCam::GetCalibratedXYMatlab:" << std::endl;
		std::cerr << "\t ... fx is 0.\n";
		return RET_FAILED;
	}
	x = (float) (z*(u-cx)/fx) ;

	// Fundamental equation: v = (fy*y)/z + cy
	if (fy == 0)
	{
		std::cerr << "VirtualRangeCam::GetCalibratedXYMatlab:" << std::endl;
		std::cerr << "\t ... fy is 0.\n";
		return RET_FAILED;
	}
	y = (float) (z*(v-cy)/fy);

	// Conversion from mm to m
	x /= 1000;
	y /= 1000;

	return RET_OK;
}

unsigned long VirtualRangeCam::GetCalibratedUV(double x, double y, double z, double& u, double& v)
{
	if(m_CalibrationMethod==MATLAB || m_CalibrationMethod==MATLAB_NO_Z)
	{
		double fx, fy, cx, cy;
		fx = m_intrinsicMatrix.at<double>(0, 0);
		fy = m_intrinsicMatrix.at<double>(1, 1);

		cx = m_intrinsicMatrix.at<double>(0, 2);
		cy = m_intrinsicMatrix.at<double>(1, 2);

		// Conversion from m to mm
		x *= 1000;
		y *= 1000;
		z *= 1000;

		// Fundamental equation: u = (fx*x)/z + cx
		if (z == 0)
		{
			std::cerr << "ERROR - VirtualRangeCam::GetCalibratedUV:" << std::endl;
			std::cerr << "\t ... z is 0.\n";
			return RET_FAILED;
		}

		u = (fx*x)/z + cx;
		v = (fy*y)/z + cy;
	}
	else if(m_CalibrationMethod==NATIVE)
	{
		// implement me
		std::cerr << "ERROR - VirtualRangeCam::GetCalibratedUV:" << std::endl;
		std::cerr << "\t ... Function not implemented.\n";
		return RET_FAILED;
	}

	// Maybe skip this part... JBK skipped this - the calling function has to check!!

	if(u<0) u=0;
	if(u>=m_ImageWidth) u=m_ImageWidth-1;
	if(v<0) v=0;
	if(v>=m_ImageHeight) v=m_ImageHeight-1;

	return RET_OK;
}

unsigned long VirtualRangeCam::LoadParameters(const char* filename, int cameraIndex)
{
	// Load SwissRanger parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));

	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file" << std::endl;
		std::cerr << "\t ...(Check filename and syntax):" << std::endl;
		std::cerr << "\t ... " << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - VirtualRangeCam::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file" << std::endl;
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
//	BEGIN LibCameraSensors->VirtualRangeCam
//************************************************************************************
			// Tag element "VirtualRangeCam" of Xml Inifile
			TiXmlElement *p_xmlElement_Root_VirtualRangeCam = NULL;
			std::stringstream ss;
			ss << "VirtualRangeCam_" << cameraIndex;
			p_xmlElement_Root_VirtualRangeCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_VirtualRangeCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->VirtualRangeCam->CameraDataDirectory
//************************************************************************************
				// Subtag element "IntrinsicParameters" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_VirtualRangeCam->FirstChildElement( "CameraDataDirectory" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					std::string tempString;
					if ( p_xmlElement_Child->QueryValueAttribute( "relativePath", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'relativePath' of tag 'CameraDataDirectory'." << std::endl;
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

//************************************************************************************
//	BEGIN LibCameraSensors->VirtualRangeCam->CalibrationMethod
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_VirtualRangeCam->FirstChildElement( "CalibrationMethod" );
				std::string tempString;
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "name", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'name' of tag 'CalibrationMethod'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "MATLAB") m_CalibrationMethod = MATLAB;
					else if (tempString == "MATLAB_NO_Z") m_CalibrationMethod = MATLAB_NO_Z;
					else if (tempString == "NATIVE") m_CalibrationMethod = NATIVE;
					else
					{
						std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CalibrationMethod'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}
//************************************************************************************
//	END LibCameraSensors->VirtualRangeCam
//************************************************************************************
			else
			{
				std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else
		{
			std::cerr << "ERROR - VirtualRangeCam::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	std::cout << "\t ... Parsing xml calibration file ... [OK] \n";

	return RET_OK;
}
