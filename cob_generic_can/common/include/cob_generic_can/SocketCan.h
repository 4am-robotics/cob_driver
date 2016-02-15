/*****************************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology (KIT)
 *
 * Author: Denis Štogl, email: denis.stogl@kit.edu
 *
 * Date of creation: February 2016
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided
 * that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
*****************************************************************************/

#ifndef SOCKETCAN_INCLUDEDEF_H
#define SOCKETCAN_INCLUDEDEF_H
//-----------------------------------------------
#include <cob_generic_can/CanItf.h>
#include <socketcan_interface/socketcan.h>
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
    void destroy() {};
    bool transmitMsg ( CanMsg CMsg, bool bBlocking = true );
    bool receiveMsg ( CanMsg* pCMsg );
    bool receiveMsgRetry ( CanMsg* pCMsg, int iNrOfRetry );
    bool receiveMsgTimeout ( CanMsg* pCMsg, int nMicroSeconds );
    bool isObjectMode() {
        return false;
    }

private:
    // --------------- Types
    can::SocketCANInterface m_handle;
    can::CommInterface::FrameListener::Ptr frame_printer;

    bool m_bInitialized;
    bool m_bSimuEnabled;
    const char* p_cDevice;
    int m_iBaudrateVal;

    std::list<can::Frame> recived_frame;

    static const int c_iInterrupt;
    static const int c_iPort;

    bool initCAN();

    void recive_frame ( const can::Frame& frame );
    void print_error ( const can::State& state );
};
//-----------------------------------------------
#endif
