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


#ifndef CANPEAKSYS_INCLUDEDEF_H
#define CANPEAKSYS_INCLUDEDEF_H
//-----------------------------------------------
#include <cob_generic_can/CanItf.h>
#include <libpcan/libpcan.h>
#include <cob_utilities/IniFile.h>
//-----------------------------------------------

class CanPeakSys : public CanItf
{
public:
	// --------------- Interface
	CanPeakSys(const char* cIniFile);
	~CanPeakSys();
	bool init_ret();
	void init();
	void destroy() {}
	bool transmitMsg(CanMsg CMsg, bool bBlocking = true);
	bool receiveMsg(CanMsg* pCMsg);
	bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry);
	bool receiveMsgTimeout(CanMsg* pCMsg, int nSecTimeout);
	bool isObjectMode() { return false; }

private:
	// --------------- Types
	HANDLE m_handle;

	bool m_bInitialized;
	IniFile m_IniFile;
	bool m_bSimuEnabled;

	static const int c_iInterrupt;
	static const int c_iPort;
};
//-----------------------------------------------
#endif

