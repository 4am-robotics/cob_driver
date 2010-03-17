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
* Description: Abstract interface for time789-of-flight cameras.
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

/// @file AbstractRangeImagingSensor.h
/// Abstract interface for range imaging sensors.
/// @author Jan Fischer
/// @date May 2008.

#ifndef __ABSTRACTRANGEIMAGINGSENSOR_H__
#define __ABSTRACTRANGEIMAGINGSENSOR_H__

#ifdef __COB_ROS__
	#include <opencv/highgui.h>
	#include <opencv/cv.h>
	#include <opencv/cxcore.h>

	#include "tinyxml/tinyxml.h"
	#include "cob_vision_utils/ThreeDUtils.h"
	#include "cob_vision_utils/OpenCVUtils.h"
	#include "cob_vision_utils/MathUtils.h"
	#include "cob_camera_sensors/LibCameraSensorsTypes.h"
#else
	#include <highgui.h>
	#include <cv.h>
	#include <cxcore.h>

	#include "Vision/Extern/TinyXml/tinyxml.h"
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/ThreeDUtils.h"
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/OpenCVUtils.h"
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/MathUtils.h"
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorTypes.h"
#endif

#include <iostream>

#ifdef __LINUX__
#define __DLL_ABSTRACTRANGEIMAGINGSENSOR_H__
#define APIENTRY
#else
	#include <windows.h>
	#ifdef __LIBCAMERASENSORS_EXPORT__
	#define __DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ __declspec(dllexport)
	#else
	#define __DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ __declspec(dllimport)
	#endif
#endif

namespace ipa_CameraSensors {

/// @ingroup RangeCameraDriver
/// Abstract interface for range imaging sensors.
class AbstractRangeImagingSensor
{
public:

	/// Struct stores the values from the xml camera configuration file
	/// All values may also be set to AUTO or DEFAULT
	struct t_RangeCameraParameters
	{
		ipa_CameraSensors::t_cameraRole m_CameraRole;	///< Master or slave camera
		std::stringstream m_AmplitudeThreshold;		///< Setting this value will set all distance values to 0 if
													///< their amplitude is lower than the amplitude threshold
		std::stringstream m_IntegrationTime;		///< Integration time of the camera
		std::stringstream m_ModulationFrequency;	///< Modulation Frequency. The higher the frequency, the lower the measurable distance
		std::stringstream m_AcquireMode;			///< Swissranger acquire mode
		std::stringstream m_ExposureMode;			///< Exposure mode of camera
		std::stringstream m_DistanceOffset;			///< Distance offset added to each distance value
		std::stringstream m_ROI;					///< Region of interest
		std::stringstream m_LensCalibration;		///< Apply lens calibration from manufacturer

		std::stringstream m_Interface;				///< Interface, the camera is connected to (i.e. USB or ETHERNET)
		std::stringstream m_IP;						///< IP address of the camera
	};
	
	/// Destructor
	virtual ~AbstractRangeImagingSensor();

	/// Initializes Swissranger.
	/// @param directory Path to the directory of the range imaging sensor parameter file.
	/// @param cameraIndex It is possible to have several cameras of the same type on the system.
	///	       One may us the camera index to apply different configuration files to each of them.
	/// @return Return code.
	virtual unsigned long Init(std::string directory, int cameraIndex = 0) = 0;
	
	/// Opens the camera device.
	/// All camera specific parameters for opening the camera should have been 
	/// set within the <code>Init()</code> function.
	/// @return Return code.
	virtual unsigned long Open() = 0;

	/// Close camera device.
	/// @return Return code.
	virtual unsigned long Close() = 0;

	/// Function to set properties of the range imaging sensor.
	/// @param propertyID The ID of the property.
	/// @param cameraProperty The value of the property.
	/// @return Return code.
	virtual unsigned long SetProperty(t_cameraProperty* cameraProperty) =0;

	/// Function to set property defaults of the range imaging sensor.
	/// @return Return code.
	virtual unsigned long SetPropertyDefaults() =0;

	/// Function to set properties of the range imaging sensor.
	/// @param propertyID The ID of the property.
	/// @param cameraProperty The value of the property.
	/// @return Return code.
	virtual unsigned long GetProperty(t_cameraProperty* cameraProperty) =0;

	/// Acquires an image from SwissRanger camera.
	/// Data is read from the camera and put into a corresponding openCV IplImage data type.
	/// The IplImages must be allocated already and are initialized within the function.
	/// @param rangeImage OpenCV IplImage with depth information.
	/// @param grayImage OpenCV IplImage with grayscale information.
	/// @param cartesianImage OpenCV IplImage with cartesian (x,y,z) information in meters.
	/// @param getLatestFrame Set true to acquire a new image on calling instead of returning the one acquired last time
	/// @param useCalibratedZ Calibrate z values 
	/// @param grayImageType Either gray image data is filled with amplitude image or intensity image
	/// @return Return code.
	virtual unsigned long AcquireImages(IplImage* rangeImage=NULL, IplImage* intensityImage=NULL,
		IplImage* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY) = 0;

	/// Acquires an image from SwissRanger.
	/// Data is read from the camera and put into a corresponding openCV IplImage data type.
	/// The IplImages are allocated and initialized within the function.
	/// @param rangeImage OpenCV IplImage with depth information.
	/// @param grayImage OpenCV IplImage with grayscale information.
	/// @param cartesianImage OpenCV IplImage with cartesian (x,y,z) information in meters.
	/// @param getLatestFrame Set true to acquire a new image on calling instead of returning the one acquired last time
	/// @param useCalibratedZ Calibrate z values 
	/// @param grayImageType Either gray image data is filled with amplitude image or intensity image
	/// @return Return code.
	virtual unsigned long AcquireImages2(IplImage** rangeImage=NULL, IplImage** intensityImage=NULL,
		IplImage** cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY) = 0;

	/// Acquires an image from SwissRanger.
	/// This implementation is designated for people that do not use openCV image type.
	/// @param rangeImage character array with depth information.
	/// @param grayImage character array  with intensity (grayscale) information.
	/// @param cartesianImage character array  with cartesian (x,y,z) information in meters.
	/// @param getLatestFrame Set true to acquire a new image on calling instead of returning the one acquired last time
	/// @param useCalibratedZ Calibrate z values 
	/// @param grayImageType Either gray image data is filled with amplitude image or intensity image
	/// @return Return code.
	virtual unsigned long AcquireImages(int widthStepOneChannel, char* rangeImage=NULL, char* grayImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true, 
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY) = 0;

	/// Save camera parameters.
	/// Saves the on-line set parameters for the range imaging camera to a file.
	/// @param filename Configuration file name.
	/// @return Return code.
	virtual unsigned long SaveParameters(const char* filename) = 0;
	
	/// Determines if range imaging camera has successfully been initialized.
	/// @return True if camera is initialized, false otherwise.
	virtual bool isInitialized() = 0;

	/// Determines if range imaging camera camera has successfully been opened.
	/// @return True if camera is open, false otherwise.
	virtual bool isOpen() = 0;

	/// Returns the utilized calibration method.
	/// Possible methods are MATLAB or SWISSRANGER.
	/// @return The utilized calibration method.
	virtual t_CalibrationMethod GetCalibrationMethod() {return m_CalibrationMethod;}

	/// Returns the camera type.
	/// @return The camera type
	virtual t_cameraType GetCameraType() {return m_CameraType;}

	/// Returns a matrix of the camera's intrinsic parameters.
	/// @param _intrinsic_matrix The OpenCV matrix that should refer to the intrinsic parameters.
	/// @return Return code.
	virtual unsigned long GetIntrinsicParameters(CvMat** _intrinsic_matrix); 

	/// Initializes the intrinsic parameters of the camera.
	/// The following equations apply: (x,y,z) = R*(X,Y,Z) + t and
	/// x' = x/z, y' = y/z and u = fx*x' + cx, v = fy*y' + cy. The model might be extended
	/// with distortion coefficients to replace x' and y' with 
	/// x'' = x'*(1 + k1*r^2 + k2*r^4) + 2*p1*x'*y' + p2(r^2+2*x'^2)
	/// y'' = y'*(1 + k1*r^2 + k2*r^4) + p1(r^2+2*y'^2) + 2*p2*x'*y'
	/// For a detailed description see openCV camera calibration description. 
	/// @param fx The focal length in x direction expressed in pixels
	/// @param fy The focal length in y direction expressed in pixels
	/// @param cx x-coordinate of principal point
	/// @param cy y-coordinate of principal point
	/// @return Return code.
	virtual unsigned long SetIntrinsicParameters(double fx, double fy, double cx, double cy);

	/// Returns the distortion coefficients.
	/// The matrix is given by [k1, k2, p1, p2] where k1, k2 are radial distortion coefficients
	/// and p1, p2 are tangential distortion coefficients.
	/// @param _distortion_coeffs The OpenCV matrix that should refer to the distortion parameters.
	/// @return Return code.
	virtual unsigned long GetDistortionParameters(CvMat** _distortion_parameters);

	/// Initializes the distortion parameters.
	/// The following equations apply: (x,y,z) = R*(X,Y,Z) + t and
	/// x' = x/z, y' = y/z and u = fx*x' + cx, v = fy*y' + cy. The model might be extended
	/// with distortion coefficients to replace x' and y' with 
	/// x'' = x'*(1 + k1*r^2 + k2*r^4) + 2*p1*x'*y' + p2(r^2+2*x'^2)
	/// y'' = y'*(1 + k1*r^2 + k2*r^4) + p1(r^2+2*y'^2) + 2*p2*x'*y'
	/// @param k1 First order radial distortion coefficient
	/// @param k2 Second order radial distortion coefficient
	/// @param p1 First order tangential distortion coefficient
	/// @param p1 Second order tangential distortion coefficient
	/// @param width Image width
	/// @param height Image height
	/// @return Return code.
	virtual unsigned long SetDistortionParameters(double k1, double k2, double p1 , double p2, int width, int height );

	/// Removes distortion from an image.
	/// It is necessary to set the distortion coefficients prior to calling this function.
	/// @param src The distorted image.
	/// @param dst The undistorted image.
	/// @return Return code.
	virtual unsigned long RemoveDistortion(const CvArr* src, CvArr* dst);
	
	bool m_Fake;	/// Set Z value of range sensor to a fix and far distance (i.e.100m)
					///´This results in a clear shared image
	
protected:
		
	t_CalibrationMethod m_CalibrationMethod; ///< Calibration method MATLAB, MATLAB_NO_Z or SWISSRANGER
	t_RangeCameraParameters m_RangeCameraParameters; ///< Storage for xml configuration file parmeters
	t_cameraType m_CameraType; ///< Camera Type

	bool m_initialized; ///< True, when the camera has sucessfully been initialized.
	bool m_open;		///< True, when the camera has sucessfully been opend.

	unsigned int m_BufferSize; ///< Number of images, the camera buffers internally

	CvMat* m_intrinsicMatrix;		///< Intrinsic parameters [fx 0 cx; 0 fy cy; 0 0 1]
	CvMat* m_distortionParameters;	///< Distortion coefficients [k1, k2, p1=0, p2=0]

	IplImage* m_undistortMapX;		///< The output array of x coordinates for the undistortion map
	IplImage* m_undistortMapY;		///< The output array of Y coordinates for the undistortion map

private:
	
	/// Load general SR31 parameters and previously determined calibration parameters.
	/// @param filename Range imaging sensor parameter path and file name.
	/// @param cameraIndex It is possible to have several cameras of the same type on the system.
	///	       One may us the camera index to apply different configuration files to each of them
	/// @return Return code
	virtual unsigned long LoadParameters(const char* filename, int cameraIndex) = 0;
};


/// Factory function to create an object of an range imaging sensor
#ifdef __cplusplus
extern "C" {
#endif
__DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ AbstractRangeImagingSensor* APIENTRY CreateRangeImagingSensor_Swissranger();
__DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ AbstractRangeImagingSensor* APIENTRY CreateRangeImagingSensor_PMDCamCube();
__DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ AbstractRangeImagingSensor* APIENTRY CreateRangeImagingSensor_VirtualCam();
__DLL_ABSTRACTRANGEIMAGINGSENSOR_H__ void APIENTRY ReleaseRangeImagingSensor(AbstractRangeImagingSensor* rangeImagingSensor);
#ifdef __cplusplus
}
#endif

} // end namespace ipa_CameraSensors
#endif

