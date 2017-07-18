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


#include "../include/cob_camera_sensors/StdAfx.h"

#ifdef __LINUX__
#include "cob_camera_sensors/AbstractRangeImagingSensor.h"
#include "cob_vision_utils/GlobalDefines.h"
#else
#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

using namespace ipa_CameraSensors;

AbstractRangeImagingSensor::~AbstractRangeImagingSensor()
{
}

unsigned long AbstractRangeImagingSensor::SetIntrinsics(cv::Mat& intrinsicMatrix,
		cv::Mat& undistortMapX, cv::Mat& undistortMapY)
{
	m_intrinsicMatrix = intrinsicMatrix.clone();
	m_undistortMapX = undistortMapX.clone();
	m_undistortMapY = undistortMapY.clone();

	return RET_OK;
}

unsigned long AbstractRangeImagingSensor::SetPathToImages(std::string path)
{
	return ipa_Utils::RET_OK;
};
