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
* Description: Toolbox for cameras.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: July 2009
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
 

/// @file CameraSensorToolbox.h
/// Toolbox for cameras.
/// @author Jan Fischer
/// @date July 2009.

#ifndef __CAMERASENSORTOOLBOX_H__
#define __CAMERASENSORTOOLBOX_H__

#ifdef __LINUX__
#define __DLL_CAMERASENSORTOOLBOX_H__ 
#define APIENTRY
#else
	#include <windows.h>
	#ifdef __LIBCAMERASENSORS_EXPORT__
	#define __DLL_CAMERASENSORTOOLBOX_H__ __declspec(dllexport)
	#else
	#define __DLL_CAMERASENSORTOOLBOX_H__ __declspec(dllimport)
	#endif
#endif

#ifdef __COB_ROS__
#include <opencv/highgui.h>
#include <opencv/cv.h>
 
#include "tinyxml/tinyxml.h"
#include "cob_camera_sensors/LibCameraSensorsTypes.h"
#include "cob_vision_utils/OpenCVUtils.h"
#else
#include <highgui.h>
#include <cv.h>

#include "Vision/Extern/TinyXml/tinyxml.h"
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/LibCameraSensorsTypes.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/OpenCVUtils.h"
#endif

#include <map>
#include <iostream>
#include <sstream>

namespace ipa_CameraSensors {

/// A toolbox for common color cameras.
///	Provides generic functions like image undistortion.
/// Holds essential matrices like distortion, intrinsic and extrinsic matrix.
/// For each camera in the system a separate camera toolbox should be initialized.
class CameraSensorToolbox
{
	public: 
		CameraSensorToolbox();	///< Constructor.
		~CameraSensorToolbox();	///< Destructor.

		CameraSensorToolbox(const CameraSensorToolbox& cameraSensorToolbox); ///< Copy constructor

		/// Overwritten assignment operator
		CameraSensorToolbox& operator= (const CameraSensorToolbox& cameraSensorToolbox); 

		/// Release all allocated memory.
		/// @return Return code
		virtual unsigned long Release();

		/// Initialize the camera sensor toolbox.
		/// The matrices are read from the specified xml configuration file.
		/// @param directory The director where the configuration resides, with ending '/'.
		/// @param cameraType The camera type
		/// @param cameraIndex The camera index
		/// @param imageSize The Size of the image returned by the camera
		/// @return Return code
		virtual unsigned long Init(std::string directory, ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, const CvSize imageSize);

		/// Initialize the camera sensor toolbox.
		/// @param intrinsicMatrix Intrinsic parameters [fx 0 cx; 0 fy cy; 0 0 1]
		/// @param distortionParameters Distortion coefficients [k1, k2, p1=0, p2=0]
		/// @param extrinsicMatrix 3x4 matrix of the form (R|t), where R is a 3x3 rotation matrix
		/// and t a 3x1 translation vector.
		/// @param undistortMapX The output array of x coordinates for the undistortion map
		/// @param undistortMapY The output array of Y coordinates for the undistortion map
		/// @param imageSize The Size of the image returned by the camera
		/// @return Return code
		virtual unsigned long Init(const CvMat* intrinsicMatrix, const CvMat* distortionParameters, 
			const std::map<std::string, CvMat*>* extrinsicMatrices, 
			const IplImage* undistortMapX, const IplImage* undistortMapY, const CvSize imageSize);

		/// Returns a matrix of the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|t), where R desribes a 3x3 rotation matrix and
		/// t a 3x1 translation vector.
		/// @param cameraType The camera type
		/// @param cameraIndex The camera index
		/// @param _extrinsic_matrix The OpenCV matrix that refers to the extrinsic parameters of the camera.
		/// @return Return code.
		virtual unsigned long GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, CvMat** _extrinsic_matrix);

		/// Returns a matrix of the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|t), where R desribes a 3x3 rotation matrix and
		/// t a 3x1 translation vector.
		/// @param cameraType The camera type
		/// @param cameraIndex The camera index
		/// @return The OpenCV matrix that refers to the extrinsic parameters of the camera.
		virtual CvMat* GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		/// Sets the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|T), where R desribes a 3x3 rotation matrix and
		/// T a 3x1 translation vector.
		/// @param t_cameraType The camera type
		/// @param cameraIndex The camera index
		/// @param _translation 3x1 translation vector.
		/// @param _rotation 3x3 rotation matrix.
		/// @return Return code.
		virtual unsigned long SetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, 
			const CvMat* _rotation, const CvMat* _translation);

		/// Sets the camera's extrinsic parameters.
		/// The extrinsic matrix is a 4x3 matrix of the format (R|T), where R desribes a 3x3 rotation matrix and
		/// T a 3x1 translation vector.
		/// @param key The key/identifier within the map of extrinsic matrices
		/// @param _translation 3x1 translation vector.
		/// @param _rotation 3x3 rotation matrix.
		/// @return Return code.
		virtual unsigned long SetExtrinsicParameters(std::string key, 
			const CvMat* _rotation, const CvMat* _translation);

		/// Returns a matrix of the camera's intrinsic parameters.
		/// @param _intrinsic_matrix The OpenCV matrix that should refer to the intrinsic parameters.
		/// @return Return code.
		virtual unsigned long GetIntrinsicParameters(CvMat** _intrinsic_matrix); 

		/// Returns a matrix of the camera's intrinsic parameters.
		/// @return The OpenCV matrix that should refer to the intrinsic parameters.
		virtual CvMat* GetIntrinsicParameters(); 

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
		/// @param _distortion_coeffs The OpenCV matrix that refers to the distortion parameters.
		/// @return Return code
		virtual unsigned long GetDistortionParameters(CvMat** _distortion_parameters);

		/// Returns the distortion coefficients.
		/// The matrix is given by [k1, k2, p1, p2] where k1, k2 are radial distortion coefficients
		/// and p1, p2 are tangential distortion coefficients.
		/// @return The OpenCV matrix that refers to the distortion parameters.
		virtual CvMat* GetDistortionParameters();

		/// Returns the distortion map for x components
		/// For each x pixel, the undistorted location is specified within the distortion map.
		/// @return The distortion map for x components
		virtual IplImage* GetDistortionMapX();

		/// Returns the distortion map for y components
		/// For each y pixel, the undistorted location is specified within the distortion map.
		/// @return The distortion map for y components
		virtual IplImage* GetDistortionMapY();

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
		/// @return Return code.
		virtual unsigned long SetDistortionParameters(double k1, double k2, double p1 , double p2);

		/// Removes distortion from an image.
		/// It is necessary to set the distortion coefficients prior to calling this function.
		/// @param src The distorted image.
		/// @param dst The undistorted image.
		/// @return Return code.
		virtual unsigned long RemoveDistortion(const CvArr* src, CvArr* dst);

		/// Returns image coordinates (u,v) from (x,y,z) coordinates. 
		/// (x,y,z) is expressed within the cameras coordinate system.
		/// @param u image coordinate u
		/// @param v image coordinate v
		/// @param x x-coordinates in mm relative to the camera's coodinate system.
		/// @param y y-coordinates in mm relative to the camera's coodinate system.
		/// @param z z-coordinates in mm relative to the camera's coodinate system.
		/// @return Return code
		virtual unsigned long ReprojectXYZ(double x, double y, double z, int& u, int& v);

	private:
		
		/// Converts the camera type enumeration value to a string.
		/// @param cameraType The camera type as enumeration
		/// @param cameraTypeString The resulting string
		/// @return Retun code
		virtual unsigned long ConvertCameraTypeToString(ipa_CameraSensors::t_cameraType cameraType, std::string &cameraTypeString);

		/// Parses the XML configuration file, that holds the camera settings
		/// @param filename The file name and path of the configuration file
		/// @param cameraType The camera type i.e. CAM_AVTPIKE or CAM_IC
		/// @param cameraIndex The index of the camera within the configuration file
		///		   i.e. AvtPikeCam_0 or ICCam_1
		/// @return Return value
		virtual unsigned long LoadParameters(const char* filename, ipa_CameraSensors::t_cameraType cameraType, int cameraIndex);

		bool m_Initialized; ///< True, when the camera has sucessfully been initialized.

		CvMat* m_intrinsicMatrix;	///< Intrinsic parameters [fx 0 cx; 0 fy cy; 0 0 1]
		CvMat* m_distortionParameters;	///< Distortion coefficients [k1, k2, p1=0, p2=0]
		std::map<std::string, CvMat*> m_extrinsicMatrices; ///< a map of 3x4 matrix of the form (R|T),
									/// where R is a 3x3 rotation matrix and T is a 3x1 translation vector.

		IplImage* m_undistortMapX;	///< The output array of x coordinates for the undistortion map
		IplImage* m_undistortMapY;	///< The output array of Y coordinates for the undistortion map

		CvSize m_ImageSize; ///< The size of the image that is returned
};

/// Factory function to create an object of LibObjectDetector
#ifdef __cplusplus
extern "C" {
#endif

__DLL_CAMERASENSORTOOLBOX_H__ CameraSensorToolbox* APIENTRY CreateCameraSensorToolbox();
__DLL_CAMERASENSORTOOLBOX_H__ void APIENTRY ReleaseCameraSensorToolbox(CameraSensorToolbox* toolbox);

#ifdef __cplusplus
}
#endif

} // end namespace
#endif // __CAMERASENSORTOOLBOX_H__
