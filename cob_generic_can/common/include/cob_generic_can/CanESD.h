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


#ifndef CANESD_INCLUDEDEF_H
#define CANESD_INCLUDEDEF_H
//-----------------------------------------------

#include <libntcan/ntcan.h>
// general includes
#include <cob_utilities/windows.h>
#include <iostream>
#include <errno.h>

// Headers provided by other cob-packages
#include <cob_generic_can/CanItf.h>

// Headers provided by other cob-packages which should be avoided/removed
#include <cob_utilities/IniFile.h>
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
    NTCAN_HANDLE m_Handle;
    int m_LastID;
    bool m_bObjectMode;
    bool m_bIsTXError;
    Mutex m_Mutex;

    IniFile m_IniFile;

    void initIntern();

public:
    CanESD(const char* cIniFile, bool bObjectMode = false);
    ~CanESD();
    bool init_ret();
    void init(){};
    bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
    bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
    bool receiveMsg(CanMsg* pCMsg);
    bool receiveMsgTimeout(CanMsg* pCMsg, int nMicroSeconds);
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

    int canIdAddGroup(NTCAN_HANDLE handle, int id);

    std::string GetErrorStr(int ntstatus) const;
    int readEvent();
};
//-----------------------------------------------
#endif
