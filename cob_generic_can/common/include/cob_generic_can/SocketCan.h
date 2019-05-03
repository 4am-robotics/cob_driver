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


#ifndef SOCKETCAN_INCLUDEDEF_H
#define SOCKETCAN_INCLUDEDEF_H
//-----------------------------------------------
#include <boost/shared_ptr.hpp>

#include <cob_generic_can/CanItf.h>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/reader.h>
//-----------------------------------------------

class SocketCan : public CanItf
{
public:
    // --------------- Interface
    SocketCan ( const char* device, int baudrate );
    SocketCan ( const char* device );
    ~SocketCan();
    bool init_ret();
    void init();
    bool transmitMsg ( CanMsg CMsg, bool bBlocking = true );
    bool receiveMsg ( CanMsg* pCMsg );
    bool receiveMsgRetry ( CanMsg* pCMsg, int iNrOfRetry );
    bool receiveMsgTimeout ( CanMsg* pCMsg, int nMicroSecTimeout );
    bool isObjectMode() {
        return false;
    }

private:
    // --------------- Types
    can::ThreadedSocketCANInterfaceSharedPtr m_handle;
    can::BufferedReader m_reader;

    bool m_bInitialized;
    const char* p_cDevice;

    void print_error(const can::State& state);
};
//-----------------------------------------------
#endif
