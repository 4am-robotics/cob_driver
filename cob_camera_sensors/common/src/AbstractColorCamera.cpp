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
#include "cob_camera_sensors/AbstractColorCamera.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#endif

using namespace ipa_CameraSensors;

AbstractColorCamera::~AbstractColorCamera()
{
}

t_cameraType AbstractColorCamera::GetCameraType()
{
	return m_CameraType;
}

unsigned long AbstractColorCamera::TestCamera(const char* filename)
{
	std::cout << "AbstractColorCamera::TestCamera: Testing camera interface class AbstractColorCamera..." << std::endl;
	std::cout << std::endl; 
	if (!isInitialized())
	{
		std::cout << "AbstractColorCamera::TestCamera: Initializing camera device..." << std::endl;
		if (Init(filename) & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED)) 
		{
			std::cout << "AbstractColorCamera::TestCamera: Initializing camera device...          FAILED" << std::endl;
			return (RET_FAILED | RET_INIT_CAMERA_FAILED);
		}
		std::cout << "AbstractColorCamera::TestCamera: Initializing camera device...          OK" << std::endl;
	}
	std::cout << std::endl;

	if (!isOpen())
	{
		std::cout << "AbstractColorCamera::TestCamera: Opening camera device..." << std::endl;
		if (Open() & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED))
		{
			std::cout << "AbstractColorCamera::TestCamera: Opening camera device...          FAILED" << std::endl;
			return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
		}
		std::cout << "AbstractColorCamera::TestCamera: Opening camera device...          OK" << std::endl;
	}
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: Displaying camera information..." << std::endl;
	unsigned long ret = PrintCameraInformation();
	if (ret & RET_FAILED)
	{
		std::cout << "AbstractColorCamera::TestCamera: Displaying camera information...          FAILED." << std::endl;
		return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
	}
	else if (ret & RET_FUNCTION_NOT_IMPLEMENTED)
	{	
		std::cout << "AbstractColorCamera::TestCamera: Displaying camera information...          NOT IMPLEMENTED" << std::endl;  
	}
	else 
	{
		std::cout << "AbstractColorCamera::TestCamera: Displaying camera information...          OK." << std::endl;
	}
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking isInitialized()..." << std::endl;
	if (!isInitialized()) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking isInitialized()...          FAILED" << std::endl;
		return (RET_FAILED | RET_INIT_CHECK_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking isInitialized()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()..." << std::endl;
	if (!isOpen()) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          FAILED" << std::endl;
		return (RET_FAILED | RET_OPEN_CHECK_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking Close()..." << std::endl;
	if (Close() & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED)) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking Close()...          FAILED" << std::endl;
		return (RET_FAILED | RET_CLOSE_CAMERA_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking Close()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()..." << std::endl;
	if (isOpen()) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          FAILED" << std::endl;
		return (RET_FAILED | RET_OPEN_CHECK_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking isOpen()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking Open()..." << std::endl;
	if (Open() & (RET_FAILED | RET_FUNCTION_NOT_IMPLEMENTED)) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking Open()...          FAILED" << std::endl;
		return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
	}
	std::cout << "AbstractColorCamera::TestCamera: checking Open()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()..." << std::endl;
	ret = SaveParameters("testSaveParams.xml");
	if (ret & RET_FAILED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()...          FAILED" << std::endl;
		return (RET_FAILED | RET_SAVE_PARAMS_FAILED);
	}
	else if (ret & RET_FUNCTION_NOT_IMPLEMENTED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()...          NOT IMPLEMENTED" << std::endl;
	} 
	else 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SaveParams()...          OK" << std::endl;
	}
	std::cout << std::endl;

	std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()..." << std::endl;
	ret = SetPropertyDefaults();
	if (ret & RET_FAILED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()...          FAILED" << std::endl;
		return (RET_FAILED | RET_SET_PROPERTY_DEFAULTS_FAILED);
	}
	else if (ret & RET_FUNCTION_NOT_IMPLEMENTED) 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()...          NOT IMPLEMENTED" << std::endl;
	} 
	else 
	{
		std::cout << "AbstractColorCamera::TestCamera: checking SetPropertyDefaults()...          OK" << std::endl;
	}
	std::cout << std::endl;

	return RET_OK;
}
