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

#include <cob_generic_can/SocketCan.h>
#include <boost/chrono.hpp>
#include <stdlib.h>
#include <cerrno>
#include <cstring>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

SocketCan::SocketCan(const char* device, int baudrate)
{
    m_bInitialized = false;

    p_cDevice = device;
    m_handle.reset(new can::ThreadedSocketCANInterface());
}

SocketCan::SocketCan(const char* device)
{
    m_bInitialized = false;

    p_cDevice = device;
    m_handle.reset(new can::ThreadedSocketCANInterface());
}

//-----------------------------------------------
SocketCan::~SocketCan()
{
    if (m_bInitialized)
    {
        m_handle->shutdown();
    }
}

//-----------------------------------------------
bool SocketCan::init_ret()
{
    bool ret = true;
    // init() - part
    if (!m_handle->init(p_cDevice, false))
    {
        print_error(m_handle->getState());
        ret = false;
    }
    else
    {
        can::BufferedReader m_reader;
        m_reader.listen(m_handle);
        std::cout << "Reader is enabled: " << m_reader.isEnabled() << std::endl;
//         frame_printer = m_handle->createMsgListener(can::CommInterface::FrameDelegate(this, &SocketCan::recive_frame));
        ret = initCAN();
    }
    return ret;
}

//-----------------------------------------------
void SocketCan::init()
{
    if (!init_ret())
    {
        sleep(3);
        exit(0);
    }
}


//-------------------------------------------
bool SocketCan::transmitMsg(CanMsg CMsg, bool bBlocking)
{
    can::Header header(CMsg.getID(), false, false, false);
    can::Frame message(header, CMsg.getLength());
    for (int i = 0; i < CMsg.getLength(); i++)
    {
        message.data[i] = CMsg.getAt(i);
    }
    return m_handle->send(message);
}

//-------------------------------------------
bool SocketCan::receiveMsg(CanMsg* pCMsg)
{
    if (m_bInitialized == false)
    {
        return false;
    }

    bool bRet = false;
    can::Frame frame;

    if (recived_frame.size())
    {
        frame = recived_frame.front();
        recived_frame.pop_front();
        pCMsg->setID(frame.id);
        pCMsg->setLength(frame.dlc);
        pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                   frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        bRet = true;
    }
    else
    {
        std::cout << "No message recieved: " << std::endl;
    }
    return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
    if (m_bInitialized == false)
    {
        return false;
    }

    can::Frame frame;
    bool bRet = false;
    int i = 0;

    do
    {
      std::cout << "BufferedReader read: " << m_reader.read(&frame, boost::chrono::seconds(1)) << std::endl;
        if (recived_frame.size())
        {
            frame = recived_frame.front();
            recived_frame.pop_front();
            pCMsg->setID(frame.id);
            pCMsg->setLength(frame.dlc);
            pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                       frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            bRet = true;
            break;
        }
        i++;
        usleep(1000);
    }
    while ((i < iNrOfRetry && bRet != true));
    return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgTimeout(CanMsg* pCMsg, int nSecTimeout)
{
    if (m_bInitialized == false)
    {
        return false;
    }

    bool bRet = false;
    can::Frame frame;
    usleep(nSecTimeout / 1000);

    if (recived_frame.size() > 0)
    {
        frame = recived_frame.front();
        recived_frame.pop_front();
        pCMsg->setID(frame.id);
        pCMsg->setLength(frame.dlc);
        pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        bRet = true;
    }

    return bRet;
}

bool SocketCan::initCAN()
{
    m_bInitialized = true;
    bool bRet = true;
    return true;
}

void SocketCan::recive_frame(const can::Frame& frame)
{

    if (frame.is_error)
    {
        can::State state;
        print_error(state);
    }
    else
    {
        recived_frame.push_back(frame);
    }
}

void SocketCan::print_error(const can::State& state)
{
    std::string err;
    std::cout << "ERROR: state=" << std::endl;
    m_handle->translateError(state.internal_error, err);
    std::cout << "ERROR: state=" << state.driver_state << " internal_error=" << state.internal_error << "('" << err << "') asio: " << state.error_code << std::endl;
}
