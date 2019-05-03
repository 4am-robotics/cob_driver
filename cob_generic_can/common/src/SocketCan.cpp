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


#include <cob_generic_can/SocketCan.h>
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
    if (!m_handle->init(p_cDevice, false))
    {
        print_error(m_handle->getState());
        ret = false;
    }
    else
    {
        m_reader.listen((can::CommInterfaceSharedPtr)m_handle);
        m_bInitialized = true;
        bool bRet = true;
        ret = true;
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
    if (!m_bInitialized)
    {
        return false;
    }

    bool bRet = false;
    can::Frame frame;

    if (m_reader.read(&frame, boost::chrono::seconds(1)))
    {
        pCMsg->setID(frame.id);
        pCMsg->setLength(frame.dlc);
        pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                   frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        bRet = true;
    }
    return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry)
{
    if (!m_bInitialized)
    {
        return false;
    }

    can::Frame frame;
    bool bRet = false;
    int i = 0;

    do
    {
        if (m_reader.read(&frame, boost::chrono::milliseconds(10)))
        { 
            pCMsg->setID(frame.id);
            pCMsg->setLength(frame.dlc);
            pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                       frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            bRet = true;
            break;
        }
        i++;
    }
    while ((i < iNrOfRetry && bRet != true));
    return bRet;
}

//-------------------------------------------
bool SocketCan::receiveMsgTimeout(CanMsg* pCMsg, int nMicroSecTimeout)
{
    if (!m_bInitialized)
    {
        return false;
    }

    bool bRet = false;
    can::Frame frame;

    if (m_reader.read(&frame, boost::chrono::microseconds(nMicroSecTimeout)))
    {
        pCMsg->setID(frame.id);
        pCMsg->setLength(frame.dlc);
        pCMsg->set(frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
        bRet = true;
    }
    return bRet;
}

void SocketCan::print_error(const can::State& state)
{
    std::string err;
    std::cout << "ERROR: state=" << std::endl;
    m_handle->translateError(state.internal_error, err);
    std::cout << "ERROR: state=" << state.driver_state << " internal_error=" << state.internal_error << "('" << err << "') asio: " << state.error_code << std::endl;
}
