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
 * ROS stack name: cob_drivers
 * ROS package name: cob_generic_can
 * Description: This class implements the interface to an ESD can node
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: April 2010
 * ToDo: - Remove Mutex.h search for a Boost lib
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


#ifndef CANESD_INCLUDEDEF_H
#define CANESD_INCLUDEDEF_H
//-----------------------------------------------

// general includes
#include <iostream>
#include <errno.h>

// Headers provided by other cob-packages
#include <cob_generic_can/CanItf.h>
#include <libntcan/ntcan.h>

// Headers provided by other cob-packages which should be avoided/removed
#include <cob_utilities/IniFile.h>
#include <cob_utilities/windows.h>
#include <cob_utilities/Mutex.h>

//-----------------------------------------------
/**
 * Driver of the CAN controller of ESD.
 */
class CanESD : public CanItf
{
private:
	BYTE m_DeviceNr;
	BYTE m_BaudRate;
	HANDLE m_Handle;
	int m_LastID;
	bool m_bObjectMode;
	bool m_bIsTXError;
	Mutex m_Mutex;

	IniFile m_IniFile;

	void initIntern();

public:
	CanESD(const char* cIniFile, bool bObjectMode = false);
	~CanESD();
	void init(){};
	bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
	bool receiveMsg(CanMsg* pCMsg);
	bool isObjectMode() { return m_bObjectMode; }
	bool isTransmitError() { return m_bIsTXError; }
protected:

    /*!
        \fn CanESD::invert(int id)
     */
	/**
	 * Invert a give ID in 1-complement.
	 * <b>Note:</b> Only 11 bits are used, i.e. the range is from 0x00 to 0x7FF.
	 * @param id The id to be inverted.
	 * @return The inverted id.
	 */
	int invert(int id)
	{
		return (~id) & 0x7F8;
	}
	
	int canIdAddGroup(HANDLE handle, int id);

	std::string GetErrorStr(int ntstatus) const;
	int readEvent();
};
//-----------------------------------------------
#endif
