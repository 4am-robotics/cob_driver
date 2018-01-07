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


/// @file VirtualRangeCam.h
/// Virtual range camera representation.
/// @author Jan Fischer
/// @date 2009

#ifndef __IPA_VIRTUALRANGECAM_H__
#define __IPA_VIRTUALRANGECAM_H__

#include "StdAfx.h"

#ifdef __LINUX__
	#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#else
	#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#endif

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <boost/filesystem.hpp>

namespace ipa_CameraSensors {

static const int SWISSRANGER_COLUMNS = 176;
static const int SWISSRANGER_ROWS = 144;

/// @ingroup VirtualCameraDriver
/// Interface class to virtual range camera like Swissranger 3000/4000.
/// The class offers an interface to a virtual range camera, that is equal to the interface of a real range camera.
/// However, pictures are read from a directory instead of the camera.
class __DLL_LIBCAMERASENSORS__ VirtualRangeCam : public AbstractRangeImagingSensor
{
public:

	VirtualRangeCam();
	~VirtualRangeCam();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImage=NULL, char* intensityImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY_32F1);
	unsigned long AcquireImages(cv::Mat* rangeImage = 0, cv::Mat* intensityImage = 0,
		cv::Mat* cartesianImage = 0, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY_32F1);

	unsigned long GetCalibratedUV(double x, double y, double z, double& u, double& v);

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

	/// Returns the number of images in the directory
	/// @return The number of images in the directory
	int GetNumberOfImages();

	/// Function specific to virtual camera.
	/// Resets the image directory read from the configuration file.
	/// @param path The camera path
	/// @return Return code
	unsigned long SetPathToImages(std::string path);

private:
	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	/// Reads out the image width and height from the first image found in the filesystem.
	/// @param filename The name of that image.
	inline void UpdateImageDimensionsOnFirstImage(std::string filename, std::string ext=".xml");

	/// Compares the value of the iterator with ext in order to find the extension which has instances in the directory.
	/// Throws an error if different file formats are present at the same time.
	/// @param itCounter Iterator containing a file extension and a number of instances.
	/// @param ext Is empty if no extension was found before, otherwise it contains the found extension.
	inline void FindSourceImageFormat(std::map<std::string, int>::iterator& itCounter, std::string& ext);

	unsigned long GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated);
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y);

	/// Load general range camera parameters .
	/// @param filename Configuration file-path and file-name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return code
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	bool m_CoeffsInitialized;

	/// Given a 32 bit swissranger depth value, the real depth value in meteres is given by:
	/// z(u,v)=a0(u,v)+a1(u,v)*d(u,v)+a2(u,v)*d(u,v)^2
	///       +a3(u,v)*d(u,v)^3+a4(u,v)*d(u,v)^4+a5(u,v)*d(u,v)^5
	///       +a6(u,v)*d(u,v)^6;
	cv::Mat m_CoeffsA0; ///< a0 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA1; ///< a1 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA2; ///< a2 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA3; ///< a3 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA4; ///< a4 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA5; ///< a5 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA6; ///< a6 z-calibration parameters. One matrix entry corresponds to one pixel

	std::string m_CameraDataDirectory; ///< Directory where the image data resides
	int m_CameraIndex; ///< Index of the specified camera. Important, when several cameras of the same type are present

	std::vector<std::string> m_AmplitudeImageFileNames;
	std::vector<std::string> m_IntensityImageFileNames;
	std::vector<std::string> m_RangeImageFileNames ;
	std::vector<std::string> m_CoordinateImageFileNames ;

	int m_ImageWidth;  ///< Image width
	int m_ImageHeight; ///< Image height

	double m_k1, m_k2, m_p1, m_p2; ///< Distortion parameters
};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr CreateRangeImagingSensor_VirtualCam();

} // end namespace ipa_CameraSensors
#endif // __IPA_VIRTUALRANGECAM_H__


