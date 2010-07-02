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

	int amplitudeImageCounter = 0;
	int intensityImageCounter = 0;
	int coordinateImageCounter = 0;
	// Extract all image filenames from the directory
	if ( fs::exists( absoluteDirectoryName ) )
	{
		std::cout << "INFO - VirtualRangeCam::Open:" << std::endl;
		std::cout << "\t ... Parsing directory '" << absoluteDirectoryName.directory_string() << "'" << std::endl;
		IplImage* image = 0;
		fs::directory_iterator end_iter;
		for ( fs::directory_iterator dir_itr( absoluteDirectoryName ); dir_itr != end_iter; ++dir_itr )
		{
			try
			{
				if (fs::is_regular_file(dir_itr->status()))
				{
					std::string filename = dir_itr->path().string();

					if ((dir_itr->path().extension() == ".xml") &&
						filename.find( "RangeCamIntensity_32F1_" + sCameraIndex, 0 ) != std::string::npos)
					{
						++intensityImageCounter;
						//std::cout << "VirtualRangeCam::Open(): Reading '" << dir_itr->path().string() << "\n";
						m_IntensityImageFileNames.push_back(dir_itr->path().string());
						if (m_ImageWidth == -1 || m_ImageHeight == -1)
						{
							image = (IplImage*) cvLoad(m_IntensityImageFileNames.back().c_str(), 0);
							m_ImageWidth = image->width;
							m_ImageHeight = image->height;
							cvReleaseImage(&image);
						}
					}

					if ((dir_itr->path().extension() == ".xml") &&
						filename.find( "RangeCamAmplitude_32F1_" + sCameraIndex, 0 ) != std::string::npos)
					{
						++amplitudeImageCounter;
						//std::cout << "VirtualRangeCam::Open(): Reading '" << dir_itr->path().string() << "\n";
						m_AmplitudeImageFileNames.push_back(dir_itr->path().string());
						if (m_ImageWidth == -1 || m_ImageHeight == -1)
						{
							image = (IplImage*) cvLoad(m_AmplitudeImageFileNames.back().c_str(), 0);
							m_ImageWidth = image->width;
							m_ImageHeight = image->height;
							cvReleaseImage(&image);
						}
					}

					if ((dir_itr->path().extension() == ".xml") &&
						filename.find( "RangeCamCoordinate_32F3_" + sCameraIndex, 0 ) != std::string::npos)
					{
						++coordinateImageCounter;
						m_CoordinateImageFileNames.push_back(dir_itr->path().string());
						if (m_ImageWidth == -1 || m_ImageHeight == -1)
						{
							image = (IplImage*) cvLoad(m_CoordinateImageFileNames.back().c_str(), 0);
							m_ImageWidth = image->width;
							m_ImageHeight = image->height;
							cvReleaseImage(&image);
						}
						//std::cout << "VirtualRangeCam::Open(): Reading '" << dir_itr->path().string() << "\n";
					}
				}
			}
			catch ( const std::exception &ex )
			{
				std::cout << "WARNING - VirtualRangeCam::Open:" << std::endl;
				std::cout << "\t ... Exception catch of '" << ex.what() << "'" << std::endl;
			}
		}
		std::sort(m_IntensityImageFileNames.begin(),m_IntensityImageFileNames.end());
		std::sort(m_AmplitudeImageFileNames.begin(),m_AmplitudeImageFileNames.end());
		std::sort(m_CoordinateImageFileNames.begin(),m_CoordinateImageFileNames.end());
		std::cout << "INFO - VirtualRangeCam::Open:" << std::endl;
		std::cout << "\t ... Extracted '" << intensityImageCounter << "' intensity images (16 bit/value)\n";
		std::cout << "\t ... Extracted '" << amplitudeImageCounter << "' amplitude images (16 bit/value)\n";
		std::cout << "\t ... Extracted '" << coordinateImageCounter << "' coordinate images (3*16 bit/value)\n";

		if (intensityImageCounter == 0 && amplitudeImageCounter == 0)
		{
			std::cerr << "ERROR - VirtualRangeCam::Open:" << std::endl;
			std::cerr << "\t ... Could not detect any intensity or amplitude images" << std::endl;
			std::cerr << "\t ... from the specified directory." << std::endl;
			return ipa_CameraSensors::RET_FAILED;
		}

		if (coordinateImageCounter != 0 &&
			intensityImageCounter != coordinateImageCounter &&
			amplitudeImageCounter != coordinateImageCounter)
		{
			std::cerr << "ERROR - VirtualRangeCam::Open:" << std::endl;
			std::cerr << "\t ... Number of intensity, range and coordinate images must agree." << std::endl;
			return ipa_CameraSensors::RET_FAILED;
		}

		if((m_CalibrationMethod == NATIVE || m_CalibrationMethod == MATLAB_NO_Z) && coordinateImageCounter == 0 )
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
	int widthStepOneChannel = -1;

	if(rangeImage)
	{
		rangeImage->create(m_ImageHeight, m_ImageWidth, CV_32FC1);
		rangeImageData = (char*) rangeImage->data;
		widthStepOneChannel = rangeImage->step;
	}

	if(grayImage)
	{
		grayImage->create(m_ImageHeight, m_ImageWidth, CV_32FC1);
		grayImageData = (char*) grayImage->data;
		widthStepOneChannel = grayImage->step;
	}

	if(cartesianImage)
	{
		cartesianImage->create(m_ImageHeight, m_ImageWidth, CV_32FC3);
		cartesianImageData = (char*) cartesianImage->data;
		widthStepOneChannel = cartesianImage->step/3;
	}

	if (widthStepOneChannel == 0)
	{
		return RET_OK;
	}

	return AcquireImages(widthStepOneChannel, rangeImageData, grayImageData, cartesianImageData, getLatestFrame, undistort, grayImageType);

}

unsigned long VirtualRangeCam::AcquireImages(int widthStepOneChannel, char* rangeImageData, char* grayImageData, char* cartesianImageData,
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
		int widthStepRangeImage = widthStepOneChannel;

		IplImage* rangeImage = (IplImage*) cvLoad(m_RangeImageFileNames[m_ImageCounter].c_str(), 0);
		
		if (!undistort)
		{
			// put data in corresponding IPLImage structures
			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (rangeImage->imageData + row*rangeImage->widthStep);
				f_ptr_dst = (float*) (rangeImageData + row*widthStepRangeImage);

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

		cvReleaseImage(&rangeImage);
	} // End if (rangeImage)
///***********************************************************************
// Gray image based on amplitude or intensity (distorted or undistorted)
///***********************************************************************
	if(grayImageData)
	{
		float* f_ptr = 0;
		float* f_ptr_dst = 0;
		IplImage* grayImage = 0;

		if (grayImageType == ipa_CameraSensors::INTENSITY)
		{
			grayImage = (IplImage*) cvLoad(m_IntensityImageFileNames[m_ImageCounter].c_str());
		}
		else
		{
			grayImage = (IplImage*) cvLoad(m_AmplitudeImageFileNames[m_ImageCounter].c_str());
		}

		int widthStepIntensityImage = widthStepOneChannel;

		if (!undistort)
		{
			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (grayImage->imageData + row*grayImage->widthStep);
				f_ptr_dst = (float*) (grayImageData + row*widthStepIntensityImage);

				for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
				{
					f_ptr_dst[col] = f_ptr[col];
				}
			}
		}
		else
		{
			cv::Mat undistortedData(m_ImageHeight, m_ImageWidth, CV_32FC1, (float*) grayImageData);
			cv::Mat cpp_grayImage = grayImage;

			assert (!m_undistortMapX.empty() && !m_undistortMapY.empty());
			cv::remap(cpp_grayImage, undistortedData, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);
		}

		cvReleaseImage(&grayImage);
	}
///***********************************************************************
// Cartesian image (always undistorted)
///***********************************************************************
	if(cartesianImageData)
	{
		int widthStepCartesianImage = widthStepOneChannel*3;
		float x = -1;
		float y = -1;
		float zCalibrated = -1;
		float* f_ptr = 0;
		float* f_ptr_dst = 0;

		if(m_CalibrationMethod==MATLAB)
		{
			CV_Assert(false);
		}
		else if(m_CalibrationMethod==MATLAB_NO_Z)
		{
			// XYZ image is assumed to be undistorted
			// Unfortunately we have no access to the swissranger calibration
			IplImage* coordinateImage = (IplImage*) cvLoad(m_CoordinateImageFileNames[m_ImageCounter].c_str(), 0);
			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (coordinateImage->imageData + row*coordinateImage->widthStep);
				f_ptr_dst = (float*) (cartesianImageData + row*widthStepCartesianImage);

				for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
				{
					int colTimes3 = 3*col;

					zCalibrated = f_ptr[colTimes3+2];
					GetCalibratedXYMatlab(col, row, zCalibrated, x, y);

					f_ptr_dst[colTimes3] = x;
					f_ptr_dst[colTimes3 + 1] = y;
					f_ptr_dst[colTimes3 + 2] = zCalibrated;
				}
			}
			cvReleaseImage(&coordinateImage);
		}
		else if(m_CalibrationMethod==NATIVE)
		{
			IplImage* coordinateImage = (IplImage*) cvLoad(m_CoordinateImageFileNames[m_ImageCounter].c_str(), 0);
			for(unsigned int row=0; row<(unsigned int)m_ImageHeight; row++)
			{
				f_ptr = (float*) (coordinateImage->imageData + row*coordinateImage->widthStep);
				f_ptr_dst = (float*) (cartesianImageData + row*widthStepCartesianImage);

				for (unsigned int col=0; col<(unsigned int)m_ImageWidth; col++)
				{
					int colTimes3 = 3*col;

					f_ptr_dst[colTimes3] =f_ptr[colTimes3+0];;
					f_ptr_dst[colTimes3 + 1] = f_ptr[colTimes3+1];
					f_ptr_dst[colTimes3 + 2] = f_ptr[colTimes3+2];
				}
			}
			cvReleaseImage(&coordinateImage);
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
