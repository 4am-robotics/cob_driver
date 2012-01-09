/****************************************************************
 *
 * Copyright (c) 2012
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_drivers
 * ROS package name: cob_base_drive_chain
 * Description: This is a sample implementation of a can-bus with several nodes. In this case it implements the drive-chain of the Care-O-bot3 mobile base. yet, this can be used as template for using the generic_can and canopen_motor packages to implement arbitrary can-setups.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2012
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef PLATFORMPARAMS_INCLUDEDEF_H
#define PLATFORMPARAMS_INCLUDEDEF_H

//-----------------------------------------------

// general includes
#include <cob_base_drive_chain/CanCtrlPltfTypes.h>
#include <cob_generic_can/CanDeviceParams.h>
#include <cob_canopen_motor/DriveParams.h>

class PlatformParams {
public:
	PlatformParams();
	~PlatformParams();
	
	CanDeviceParams can_device_params;
	
	CanOpenIDType canopen_ids;
	
	DriveParam drive_param_w1_drive;
	DriveParam drive_param_w1_steer;
	DriveParam drive_param_w2_drive;
	DriveParam drive_param_w2_steer;
	DriveParam drive_param_w3_drive;
	DriveParam drive_param_w3_steer;
	DriveParam drive_param_w4_drive;
	DriveParam drive_param_w4_steer;
	
	
};
#endif


