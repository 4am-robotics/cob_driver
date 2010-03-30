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
 * ROS stack name: cob3_common
 * ROS package name: generic_can
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Feb 2009
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


#ifndef CANITF_INCLUDEDEF_H
#define CANITF_INCLUDEDEF_H
//-----------------------------------------------
#include <generic_can/CanMsg.h>
//-----------------------------------------------

/**
 * General interface of the CAN bus.
 * \ingroup DriversCanModul	
 */
class CanItf
{
public:
	enum CanItfType {
		CAN_PEAK = 0,
		CAN_PEAK_USN = 1,
		CAN_ESD = 2,
		CAN_DUMMY = 3,
		CAN_BECKHOFF = 4
	};
	
	/**
	 * The destructor does not necessarily have to be overwritten.
	 * But it makes sense to close any resources like handles.
	 */
	virtual ~CanItf() {
	}
	
	/**
	 * Initializes the CAN bus.
	 */
	virtual void init() = 0;

	/**
	 * Sends a CAN message.
	 * @param pCMsg CAN message
	 * @param bBlocking specifies whether send should be blocking or non-blocking
	 */
	virtual bool transmitMsg(CanMsg CMsg, bool bBlocking = true) = 0;

	/**
	 * Reads a CAN message.
	 * @return true if a message is available 
	 */
	virtual bool receiveMsg(CanMsg* pCMsg) = 0;

	/**
	 * Reads a CAN message.
	 * The function blocks between the attempts.
	 * @param pCMsg CAN message
	 * @param iNrOfRetry number of retries
	 * @return true if a message is available
	 */
	virtual bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry) = 0;

	/**
	 * Check if the current CAN interface was opened on OBJECT mode.
	 * @return true if opened in OBJECT mode, false if not.
	 */
	virtual bool isObjectMode() = 0;
	
	/**
	 * Set the CAN interface type. This is necessary to implement
	 * a proper CAN bus simulation.
	 * @param iType The CAN interface type.
	 */
	void setCanItfType(CanItfType iType) { m_iCanItfType = iType; }
	
	/**
	 * Get the CAN interface type. This is necessary to implement
	 * a proper CAN bus simulation.
	 * @return The CAN interface type.
	 */
	CanItfType getCanItfType() { return m_iCanItfType; }
	
private:
	/// The CAN interface type.
	CanItfType m_iCanItfType;
};
//-----------------------------------------------

#endif
