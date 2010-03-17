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
#include "cob_camera_sensors/CameraSensorToolbox.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/CameraSensorToolbox.h"
#endif

using namespace ipa_CameraSensors;

#ifdef __cplusplus
extern "C" {
#endif
__DLL_CAMERASENSORTOOLBOX_H__ void APIENTRY ReleaseCameraSensorToolbox(CameraSensorToolbox* toolbox)
{
	delete toolbox;
}
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
__DLL_CAMERASENSORTOOLBOX_H__ CameraSensorToolbox* APIENTRY CreateCameraSensorToolbox()
{
	return (new CameraSensorToolbox());
}
#ifdef __cplusplus
}
#endif

CameraSensorToolbox::CameraSensorToolbox()
{
	m_intrinsicMatrix = 0;
	m_distortionParameters = 0;
	m_undistortMapX = 0;
	m_undistortMapY = 0;

	m_Initialized = false;
}

CameraSensorToolbox::~CameraSensorToolbox()
{
	Release();
}

unsigned long CameraSensorToolbox::Release()
{
	if (m_intrinsicMatrix)
	{
		cvReleaseMat(&m_intrinsicMatrix);
		m_intrinsicMatrix = 0;
	}
	if (m_distortionParameters)
	{
		cvReleaseMat(&m_distortionParameters);
		m_distortionParameters = 0;
	}
	if (m_undistortMapX)
	{
		cvReleaseImage(&m_undistortMapX);
		m_undistortMapX = 0;
	}
	if (m_undistortMapY)
	{
		cvReleaseImage(&m_undistortMapY);
		m_undistortMapY = 0;
	}

	std::map<std::string, CvMat*>::iterator matrixIterator;
	while (!m_extrinsicMatrices.empty())
	{
		matrixIterator = m_extrinsicMatrices.begin();
		m_extrinsicMatrices.erase(matrixIterator);
	}
	return RET_OK;
}

CameraSensorToolbox::CameraSensorToolbox(const CameraSensorToolbox& cst)
{
	if(cst.m_intrinsicMatrix != 0)
	{
		m_intrinsicMatrix = cvCloneMat(cst.m_intrinsicMatrix);
	} else m_intrinsicMatrix = 0;

	if(cst.m_distortionParameters != 0)
	{
		m_distortionParameters = cvCloneMat(cst.m_distortionParameters);
	} else m_distortionParameters = 0;

	if(cst.m_undistortMapX != 0)
	{
		m_undistortMapX = cvCloneImage(cst.m_undistortMapX);
	} else m_undistortMapX = 0;

	if(cst.m_undistortMapY != 0)
	{
		m_undistortMapY = cvCloneImage(cst.m_undistortMapY);
	} else m_undistortMapY = 0;

	std::map<std::string, CvMat*>::const_iterator matrixIterator;
	for ( matrixIterator=cst.m_extrinsicMatrices.begin() ; matrixIterator != cst.m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	m_Initialized = cst.m_Initialized;
}

CameraSensorToolbox& CameraSensorToolbox::operator=(const CameraSensorToolbox& cst)
{
	/// Check for self-assignment
	if (this==&cst)
	{
		 return *this;
	}

	if(cst.m_intrinsicMatrix != 0)
	{
		m_intrinsicMatrix = cvCloneMat(cst.m_intrinsicMatrix);
	} else m_intrinsicMatrix = 0;

	if(cst.m_distortionParameters != 0)
	{
		m_distortionParameters = cvCloneMat(cst.m_distortionParameters);
	} else m_distortionParameters = 0;

	if(cst.m_undistortMapX != 0)
	{
		m_undistortMapX = cvCloneImage(cst.m_undistortMapX);
	} else m_undistortMapX = 0;

	if(cst.m_undistortMapY != 0)
	{
		m_undistortMapY = cvCloneImage(cst.m_undistortMapY);
	} else m_undistortMapY = 0;

	std::map<std::string, CvMat*>::const_iterator matrixIterator;
	for ( matrixIterator=cst.m_extrinsicMatrices.begin() ; matrixIterator != cst.m_extrinsicMatrices.end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	m_Initialized = cst.m_Initialized;

	return *this;
}

unsigned long CameraSensorToolbox::Init(std::string directory, ipa_CameraSensors::t_cameraType cameraType,
										int cameraIndex, const CvSize imageSize)
{
	Release();

	m_ImageSize = imageSize;

	std::string iniFileNameAndPath = directory;
	iniFileNameAndPath += "cameraSensorsIni.xml";
	if (LoadParameters(iniFileNameAndPath.c_str(), cameraType, cameraIndex) & RET_FAILED)
	{
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);
	}

	m_Initialized = true;
	return RET_OK;
}

unsigned long CameraSensorToolbox::Init(const CvMat* intrinsicMatrix,const CvMat* distortionParameters,
										const std::map<std::string, CvMat*>* extrinsicMatrices,
										const IplImage* undistortMapX, const IplImage* undistortMapY,
										const CvSize imageSize)
{
	Release();

	m_ImageSize = imageSize;

	m_intrinsicMatrix = cvCloneMat(intrinsicMatrix);
	m_distortionParameters = cvCloneMat(distortionParameters);

	std::map<std::string, CvMat*>::const_iterator matrixIterator;
	for ( matrixIterator=extrinsicMatrices->begin() ; matrixIterator != extrinsicMatrices->end(); matrixIterator++ )
	{
		m_extrinsicMatrices[matrixIterator->first] = cvCloneMat(matrixIterator->second);
	}

	m_undistortMapX = cvCloneImage(undistortMapX);
	m_undistortMapY = cvCloneImage(undistortMapY);

	m_Initialized = true;
	return RET_OK;
}

unsigned long CameraSensorToolbox::ConvertCameraTypeToString(ipa_CameraSensors::t_cameraType cameraType, std::string &cameraTypeString)
{
	switch (cameraType)
	{
	case CAM_IC:
		cameraTypeString = "ICCam";
		break;
	case CAM_AVTPIKE:
		cameraTypeString = "AVTPikeCam";
		break;
	case CAM_AXIS:
		cameraTypeString = "AxisCam";
		break;
	case CAM_VIRTUALCOLOR:
		cameraTypeString = "VirtualColorCam";
		break;
	case CAM_SWISSRANGER:
		cameraTypeString = "Swissranger";
		break;
	case CAM_PMDCAMCUBE:
		cameraTypeString = "PMDCamCube";
		break;
	case CAM_VIRTUALRANGE:
		cameraTypeString = "VirtualRangeCam";
		break;
	default:
		std::cerr << "ERROR - CameraSensorToolbox::ConvertCameraTypeToString:" << std::endl;
		std::cerr << "\t ... Camera type " << cameraType << " unspecified." << std::endl;
		return RET_FAILED;
	}

	return RET_OK;
}

CvMat* CameraSensorToolbox::GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	CvMat* extrinsicMatrix = 0;
	if (GetExtrinsicParameters(cameraType, cameraIndex, &extrinsicMatrix) & RET_FAILED)
	{
		return 0;
	}
	return extrinsicMatrix;
}

unsigned long CameraSensorToolbox::GetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex, CvMat** _extrinsic_matrix)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	if (m_extrinsicMatrices.find(ss.str()) == m_extrinsicMatrices.end())
	{
		std::cout << "ERROR - CameraSensorToolbox::GetExtrinsicParameters:" << std::endl;
		std::cout << "\t ... Extrinsic matrix to '" << ss.str() << "' not specified\n";
		return RET_FAILED;
	}
	else
	{
		*_extrinsic_matrix = cvCloneMat(m_extrinsicMatrices[ss.str()]);
		return RET_OK;
	}
}

unsigned long CameraSensorToolbox::SetExtrinsicParameters(ipa_CameraSensors::t_cameraType cameraType, int cameraIndex,
														  const CvMat* _rotation, const CvMat* _translation)
{
	std::stringstream ss;
	std::string extrinsicMapName = "";

	ConvertCameraTypeToString(cameraType, extrinsicMapName);
	ss << extrinsicMapName << "_" << cameraIndex;

	return SetExtrinsicParameters(ss.str(), _rotation, _translation);
}

unsigned long CameraSensorToolbox::SetExtrinsicParameters(std::string key,
														  const CvMat* _rotation, const CvMat* _translation)
{
	std::map<std::string, CvMat*>::iterator iterator;
	iterator = m_extrinsicMatrices.find(key);
	if (iterator != m_extrinsicMatrices.end())
	{
		cvReleaseMat(&iterator->second);
		m_extrinsicMatrices.erase(iterator);
	}

	CvMat* extrinsicMatrix = cvCreateMatHeader( 3, 4, CV_64FC1 );
	cvCreateData( extrinsicMatrix );
	cvSet(extrinsicMatrix, cvRealScalar(0), NULL);

	cvmSet(extrinsicMatrix, 0, 0, cvmGet(_rotation, 0, 0));
	cvmSet(extrinsicMatrix, 0, 1, cvmGet(_rotation, 0, 1));
	cvmSet(extrinsicMatrix, 0, 2, cvmGet(_rotation, 0, 2));
	cvmSet(extrinsicMatrix, 1, 0, cvmGet(_rotation, 1, 0));
	cvmSet(extrinsicMatrix, 1, 1, cvmGet(_rotation, 1, 1));
	cvmSet(extrinsicMatrix, 1, 2, cvmGet(_rotation, 1, 2));
	cvmSet(extrinsicMatrix, 2, 0, cvmGet(_rotation, 2, 0));
	cvmSet(extrinsicMatrix, 2, 1, cvmGet(_rotation, 2, 1));
	cvmSet(extrinsicMatrix, 2, 2, cvmGet(_rotation, 2, 2));

	cvmSet(extrinsicMatrix, 0, 3, cvmGet(_translation, 0, 0));
	cvmSet(extrinsicMatrix, 1, 3, cvmGet(_translation, 1, 0));
	cvmSet(extrinsicMatrix, 2, 3, cvmGet(_translation, 2, 0));

	m_extrinsicMatrices[key] = extrinsicMatrix;

	return RET_OK;
}

unsigned long CameraSensorToolbox::SetIntrinsicParameters(double fx, double fy, double cx, double cy)
{ //[fx 0 cx; 0 fy cy; 0 0 1]

	if (m_intrinsicMatrix == NULL)
	{
		m_intrinsicMatrix = cvCreateMatHeader( 3, 3, CV_64FC1 );
		cvCreateData( m_intrinsicMatrix );
		cvSet(m_intrinsicMatrix, cvRealScalar(0), NULL);
	}

	cvmSet(m_intrinsicMatrix,0,0,fx);
	cvmSet(m_intrinsicMatrix,1,1,fy);
	cvmSet(m_intrinsicMatrix,0,2,cx);
	cvmSet(m_intrinsicMatrix,1,2,cy);
	cvmSet(m_intrinsicMatrix,2,2, 1.f);

	return RET_OK;
}

CvMat* CameraSensorToolbox::GetIntrinsicParameters()
{
	CvMat* intrinsicMatrix = 0;
	if (GetIntrinsicParameters(&intrinsicMatrix) & RET_FAILED)
	{
		return 0;
	}
	return intrinsicMatrix;
}

unsigned long CameraSensorToolbox::GetIntrinsicParameters(CvMat** _intrinsic_matrix)
{
	if (m_intrinsicMatrix == 0)
	{
		return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
	}
	else
	{
		*_intrinsic_matrix = cvCreateMat( 3, 3, CV_64FC1 );
		cvCopy(m_intrinsicMatrix, *_intrinsic_matrix);
		return RET_OK;
	}
}

unsigned long CameraSensorToolbox::SetDistortionParameters(double k1, double k2, double p1, double p2)
{

	if (m_intrinsicMatrix == 0)
	{
		std::cerr << "ERROR - CameraSensorToolbox::SetDistortionParameters:\n";
		std::cerr << "\t ... Could not init undistortion matrix because intrinsic matrix is has to be set first.";
		return RET_FAILED;
	}

	if (m_distortionParameters == NULL)
	{
		m_distortionParameters = cvCreateMatHeader( 1, 4, CV_64FC1 );//Initialisierung
		cvCreateData( m_distortionParameters );
		cvSet(m_distortionParameters,cvRealScalar(0), NULL);//Defaultwerte:0

		m_undistortMapX = cvCreateImage(m_ImageSize, IPL_DEPTH_32F, 1);
		m_undistortMapY = cvCreateImage(m_ImageSize, IPL_DEPTH_32F, 1);
	}

	cvmSet(m_distortionParameters,0,0,k1);
	cvmSet(m_distortionParameters,0,1,k2);

	cvmSet(m_distortionParameters,0,2,p1);
	cvmSet(m_distortionParameters,0,3,p2);

	ipa_Utils::InitUndistortMap(m_intrinsicMatrix, m_distortionParameters, m_undistortMapX, m_undistortMapY);

	return RET_OK;
}


CvMat* CameraSensorToolbox::GetDistortionParameters()
{
	CvMat* distortionParameters = 0;
	if (GetDistortionParameters(&distortionParameters) & RET_FAILED)
	{
		return 0;
	}
	return distortionParameters;
}

unsigned long CameraSensorToolbox::GetDistortionParameters(CvMat** _distortion_coeffs)
{
	if (m_distortionParameters == 0)
	{
		return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
	}
	else
	{
		*_distortion_coeffs = cvCreateMat( 1, 4, CV_64FC1 );
		cvCopy(m_distortionParameters, *_distortion_coeffs);
		return RET_OK;
	}
}

IplImage* CameraSensorToolbox::GetDistortionMapX()
{
	return m_undistortMapX;
}

IplImage* CameraSensorToolbox::GetDistortionMapY()
{
	return m_undistortMapY;
}

unsigned long CameraSensorToolbox::RemoveDistortion(const CvArr* src, CvArr* dst)
{
	if ((m_intrinsicMatrix != NULL) && (m_distortionParameters != NULL))
	{
		cvRemap(src, dst, m_undistortMapX, m_undistortMapY);
		return RET_OK;
	}

	return (RET_FAILED | RET_MISSING_INTRINSIC_DISTORTION_PARAMS);
}

unsigned long CameraSensorToolbox::ReprojectXYZ(double x, double y, double z, int& u, int& v)
{
	CvMat* UV1 = cvCreateMat(3, 1, CV_64FC1);
	CvMat* XYZ = cvCreateMat(3, 1, CV_64FC1);

	cvSetZero( UV1 );
	cvSetZero( XYZ );

	x *= 1000;
	y *= 1000;
	z *= 1000;

	x = x/z;
	y = y/z;
	z = 1;

	cvmSet(XYZ, 0, 0, x);
	cvmSet(XYZ, 1, 0, y);
	cvmSet(XYZ, 2, 0, z);

	/// Fundamental equation: u = (fx*x)/z + cx
	if (z == 0)
	{
		std::cerr << "ERROR - CameraSensorToolbox::ReprojectXYZ" << std::endl;
		std::cerr << "\t ... z value is 0.\n";
		return RET_FAILED;
	}

	cvMatMulAdd( m_intrinsicMatrix, XYZ, 0, UV1 );

	u = cvRound(cvmGet(UV1, 0, 0));
	v = cvRound(cvmGet(UV1, 1, 0));

	cvReleaseMat(&UV1);
	cvReleaseMat(&XYZ);

	return RET_OK;
}


unsigned long CameraSensorToolbox::LoadParameters(const char* filename, ipa_CameraSensors::t_cameraType cameraType, int cameraIndex)
{
	std::stringstream ss;
	std::string xmlTagName = "";

	ConvertCameraTypeToString(cameraType, xmlTagName);
	ss << xmlTagName << "_" << cameraIndex;

	TiXmlDocument* p_configXmlDocument = new TiXmlDocument( filename );
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
		std::cerr << "\t ...  Error while loading xml configuration file (Check filename and syntax of the file):\n" << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - CameraSensorsToolbox::LoadParameters:" << std::endl;
	std::cout << "\t ...  Parsing xml configuration file:" << std::endl;
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
//	BEGIN LibCameraSensors->XYZCam
//************************************************************************************
			// Tag element "CameraSensorsToolbox" of Xml Inifile
			TiXmlElement *p_xmlElement_Root_AVTPikeCam = NULL;
			p_xmlElement_Root_AVTPikeCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_AVTPikeCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->IntrinsicParameters
//************************************************************************************
				// Subtag element "IntrinsicParameters" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "IntrinsicParameters" );
				if ( p_xmlElement_Child )
				{
					double fx, fy, cx, cy;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "fx", &fx ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'fx' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "fy", &fy ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'fy' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "cx", &cx ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'cx' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "cy", &cy ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'cy' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					SetIntrinsicParameters(fx, fy, cx, cy);
				}
				else
				{
					std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
					std::cerr << "\t ...  Can't find tag 'IntrinsicParameters'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->DistortionCoeffs
//************************************************************************************
				// Subtag element "DistortionCoeffs " of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "DistortionCoeffs" );
				if ( p_xmlElement_Child )
				{
					double k1, k2, p1, p2;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "k1", &k1 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'k1' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "k2", &k2 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'k2' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "p1", &p1 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'p1' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "p2", &p2 ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'p2' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					SetDistortionParameters(k1, k2, p1, p2);
				}
				else
				{
					std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
					std::cerr << "\t ...  Can't find tag 'DistortionCoeffs '." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->ExtrinsicParameters
//************************************************************************************
				// Subtag element "Translation" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_AVTPikeCam->FirstChildElement( "ExtrinsicParameters" );
				if ( p_xmlElement_Child )
				{
					TiXmlElement *p_xmlElement_Extrinsics = 0;
					TiXmlElement *p_xmlElement_Extrinsics_Child = 0;
					/// Iterate all children (extrinsic matrices)
					for( p_xmlElement_Extrinsics = p_xmlElement_Child->FirstChildElement();
						p_xmlElement_Extrinsics;
						p_xmlElement_Extrinsics = p_xmlElement_Extrinsics->NextSiblingElement())
					{

//************************************************************************************
//	BEGIN LibCameraSensors->CameraSensorsToolbox->Translation
//************************************************************************************
						// Subtag element "Translation" of Xml Inifile
						CvMat* extrinsicTranslation = cvCreateMat(3, 1, CV_64FC1);
						p_xmlElement_Extrinsics_Child = NULL;
						p_xmlElement_Extrinsics_Child = p_xmlElement_Extrinsics->FirstChildElement( "Translation" );
						if ( p_xmlElement_Extrinsics_Child )
						{
							double x, y, z;
							// read and save value of attribute
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x", &x ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x' of tag 'Translation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "y", &y ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'y' of tag 'Translation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "z", &z ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'z' of tag 'Translation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							cvmSet(extrinsicTranslation, 0, 0, x);
							cvmSet(extrinsicTranslation, 1, 0, y);
							cvmSet(extrinsicTranslation, 2, 0, z);
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'Translation'." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

		//************************************************************************************
		//	BEGIN LibCameraSensors->CameraSensorsToolbox->Rotation
		//************************************************************************************
						// Subtag element "Rotation" of Xml Inifile
						CvMat* extrinsicRotation = cvCreateMat(3, 3, CV_64FC1);
						p_xmlElement_Extrinsics_Child = NULL;
						p_xmlElement_Extrinsics_Child = p_xmlElement_Extrinsics->FirstChildElement( "Rotation" );
						if ( p_xmlElement_Extrinsics_Child )
						{
							double x11, x12, x13;
							double x21, x22, x23;
							double x31, x32, x33;
							// read and save value of attribute
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x11", &x11 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:R" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x11' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x12", &x12 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x12' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x13", &x13 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x13' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x21", &x21 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x21' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x22", &x22 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x22' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x23", &x23 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x23' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x31", &x31 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x31' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x32", &x32 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x32' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							if ( p_xmlElement_Extrinsics_Child->QueryValueAttribute( "x33", &x33 ) != TIXML_SUCCESS)
							{
								std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
								std::cerr << "\t ...  Can't find attribute 'x33' of tag 'Rotation'." << std::endl;
								return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
							}
							cvmSet(extrinsicRotation, 0, 0, x11);
							cvmSet(extrinsicRotation, 0, 1, x12);
							cvmSet(extrinsicRotation, 0, 2, x13);
							cvmSet(extrinsicRotation, 1, 0, x21);
							cvmSet(extrinsicRotation, 1, 1, x22);
							cvmSet(extrinsicRotation, 1, 2, x23);
							cvmSet(extrinsicRotation, 2, 0, x31);
							cvmSet(extrinsicRotation, 2, 1, x32);
							cvmSet(extrinsicRotation, 2, 2, x33);
						}
						else
						{
							std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
							std::cerr << "\t ...  Can't find tag 'Rotation'." << std::endl;
							return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
						}

						SetExtrinsicParameters(p_xmlElement_Extrinsics->Value(), extrinsicRotation, extrinsicTranslation);
						cvReleaseMat(&extrinsicTranslation);
						cvReleaseMat(&extrinsicRotation);
					} /// End 'extrinsic' for loop
				}
				else
				{
					std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'ExtrinsicParameters'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->CameraSensorsToolbox
//************************************************************************************
			else
			{
				std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else
		{
			std::cerr << "ERROR - CameraSensorsToolbox::LoadParameters:" << std::endl;
			std::cerr << "\t ...  Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	return RET_OK;
}
