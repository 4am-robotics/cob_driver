/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Neobotix GmbH
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Neobotix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef CANMSG_INCLUDEDEF_H
#define CANMSG_INCLUDEDEF_H

//-----------------------------------------------

#include <iostream>
#include <cob_utilities/TimeStamp.h>
#include <cob_generic_can/stdDef.h>

//-----------------------------------------------

/**
 * Represents a CAN message.
 * \ingroup DriversCanModul	
 */
class CanMsg
{
public:
	int m_iID;
	int m_iLen;
	int m_iType;

private:
	/**
	 * A CAN message consists of eight chars.
	 */
	unsigned char m_bDat[8];

public:
	/**
	 * Default constructor.
	 */
	CanMsg()
	{
		m_iID = 0;
		m_iLen = 8;
		m_iType = 0x00;
	}

	/**
	 * Sets the chars to the telegram.
	 */
	void set(char Data0, char Data1, char Data2, char Data3, char Data4, char Data5, char Data6, char Data7)
	{
		m_bDat[0] = Data0;
		m_bDat[1] = Data1;
		m_bDat[2] = Data2;
		m_bDat[3] = Data3;
		m_bDat[4] = Data4;
		m_bDat[5] = Data5;
		m_bDat[6] = Data6;
		m_bDat[7] = Data7;
	}

	/**
	 * Gets the chars of the telegram.
	 */
	void get(char* pData0, char* pData1, char* pData2, char* pData3, char* pData4, char* pData5, char* pData6, char* pData7)
	{
		*pData0 = m_bDat[0];
		*pData1 = m_bDat[1];
		*pData2 = m_bDat[2];
		*pData3 = m_bDat[3];
		*pData4 = m_bDat[4];
		*pData5 = m_bDat[5];
		*pData6 = m_bDat[6];
		*pData7 = m_bDat[7];
	}

	/**
	 * Returns a specific char of the telegram.
	 * @param iNr number of the char.
	 */
	int getAt(int iNr)
	{
		return m_bDat[iNr];
	}

	/**
	 * Prints the telegram to the standard output.
	 * @deprecated function uses a spetific format of the telegram.
	 */
	int printCanIdentMsgStatus()
	{
		if(getStatus() == 0)
		{
			std::cout << "ID= " << m_iID << "  " << "Cmd= " << getCmd() << "   " << "Msg_OK" << std::endl;
			return 0;
		}
		else
		{
			std::cout << "ID= " << m_iID << "  " << "Cmd= " << getCmd() << "   " << "Msg_Error" << std::endl;
			return -1;
		}
	}

	/**
	 * Prints the telegram to the standard output.
	 */
	void print()
	{
		std::cout << std::hex << std::showbase << "                id= " << m_iID << " type= " << m_iType << 
			" len= " << m_iLen << " msg= " <<
			(int)m_bDat[0] << " " << (int)m_bDat[1] << " " << (int)m_bDat[2] << " " << (int)m_bDat[3] << " " <<
			(int)m_bDat[4] << " " << (int)m_bDat[5] << " " << (int)m_bDat[6] << " " << (int)m_bDat[7] << std::endl;
	}

	/**
	 * @deprecated function uses a specific format of the telegram.
	 */
	int getStatus()
	{
		//bit 0 and bit 1 contain MsgStatus
		return (int)(m_bDat[7] & 0x0003);
	}

	/**
	 * @deprecated function uses a specific format of the telegram.
	 */
	int getCmd()
	{
		return (m_bDat[7] >> 2);
	}
};

/**
 *	Represents a CAN message with a timestamp.
 *  \ingroup DriversCanModul	
 */
class CANTimedMessage
{
	public:
		/**
			The timestamp associated with the CAN message.
		*/
		double dTimeStamp;

		/**
			The CAN message.
		*/
		CanMsg *pCanMsg;
};


//-----------------------------------------------
#endif
