/* -----------------------------------------------------------------------------------
 *
 *		Copyright (c) 2005 Neobotix (www.neobotix.de)
 *
 *      This software is allowed to be used and modified only in association with a Neobotix
 *      robot platform. It is allowed to include the software into applications and
 *      to distribute it with a Neobotix robot platform. 
 *      It is not allowed to distribute the software without a Neobotix robot platform. 
 *
 *      This software is provided WITHOUT ANY WARRANTY; without even the
 *		implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *		PURPOSE. See the Neobotix License (Version 1.0) for more details.
 *
 *      You should have received a copy of the Neobotix License
 *      along with this software; if not, write to the 
 *      Gesellschaft fuer Produktionssysteme, Neobotix, Nobelstrasse 12, 70569 Stuttgart, Germany
 *
 * -----------------------------------------------------------------------------------
 */


#ifndef CANESD_INCLUDEDEF_H
#define CANESD_INCLUDEDEF_H
//-----------------------------------------------
//#include <windows.h>
#include <iostream>
#include <cstdio>
#include <errno.h>
#include "CanItf.h"
#include <ntcan.h>
//#include <Neobotix/Utilities/IniFile.h>
#include "Mutex.h"

//-----------------------------------------------
/**
 * Driver of the CAN controller of ESD.
 * \ingroup LinuxDriversCanModul
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

	//IniFile m_IniFile;

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
	//return (~id) & 0x7FF;
		return (~id) & 0x7F8;
	}
	
	int canIdAddGroup(HANDLE handle, int id);

	std::string GetErrorStr(int ntstatus) const;
	int readEvent();
};
//-----------------------------------------------
#endif
