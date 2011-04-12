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


#ifndef CANMSG_INCLUDEDEF_H
#define CANMSG_INCLUDEDEF_H
//-----------------------------------------------

//#include "stdafx.h"
#include <stdint.h>
#define BYTE uint8_t 

#include <iostream>
//-----------------------------------------------

/**
 * Represents a CAN message.
 * \ingroup DriversCanModul	
 */
class CanMsg
{
public:
	/// @todo This should be private.
	int m_iID;
	/// @todo This should be private.
	int m_iLen;
	/// @todo This should be private.
	int m_iType;

protected:
	/**
	 * A CAN message consists of eight bytes.
	 */
	BYTE m_bDat[8];

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
	 * Sets the bytes to the telegram.
	 */
	void set(BYTE Data0=0, BYTE Data1=0, BYTE Data2=0, BYTE Data3=0, BYTE Data4=0, BYTE Data5=0, BYTE Data6=0, BYTE Data7=0)
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
	 * Set the byte at the given position.
	 */
	void setAt(BYTE data, int iNr)
	{
		m_bDat[iNr] = data;
	}

	/**
	 * Gets the bytes of the telegram.
	 */
	void get(BYTE* pData0, BYTE* pData1, BYTE* pData2, BYTE* pData3, BYTE* pData4, BYTE* pData5, BYTE* pData6, BYTE* pData7)
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
	 * Returns a spezific byte of the telegram.
	 * @param iNr number of the byte.
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
			std::cout <<"ID= " << m_iID << "  " << "Cmd= " << getCmd() << "   " << "Msg_OK" << std::endl;
			return 0;
		}
		else
		{
			std::cout << "ID= " << m_iID << "  " << "Cmd= " << getCmd() << "   " << "Msg_Error" << std::endl;
			return -1;
		}
	}

	/**
	 * Prints the telegram.
	 */
	void print()
	{
		std::cout << "id= " << m_iID << " type= " << m_iType << " len= " << m_iLen << " data= " <<
			(int)m_bDat[0] << " " << (int)m_bDat[1] << " " << (int)m_bDat[2] << " " << (int)m_bDat[3] << " " <<
			(int)m_bDat[4] << " " << (int)m_bDat[5] << " " << (int)m_bDat[6] << " " << (int)m_bDat[7] << std::endl;
	}

	/**
	 * @deprecated function uses a spetific format of the telegram.
	 */
	int getStatus()
	{
		//bit 0 and bit 1 contain MsgStatus
		return (int)(m_bDat[7] & 0x0003);
	}

	/**
	 * @deprecated function uses a spetific format of the telegram.
	 */
	int getCmd()
	{
		return (m_bDat[7] >> 2);
	}
	
	/**
	 * Get the identifier stored in this message structure.
	 * @return the message identifier.
	 */
	int getID()
	{
		return m_iID;
	}

	/**
	 * Set the message identifier within this message structure.
	 * @param id The message identifier. Its value must be in the range [0..2047], i.e.
	 * 29-bit identifiers are not supported here.
	 */
	void setID(int id)
	{
		if( (0 <= id) && (id <= 2047) )
			m_iID = id;
	}

	/**
	 * Get the message length set within this data structure.
	 * @return The message length in the range [0..8].
	 */
	int getLength()
	{
		return m_iLen;
	}

	/**
	 * Set the message length within this message structure.
	 * @param len The message length. Its value must be in the range [0..8].
	 */
	void setLength(int len)
	{
		if( (0 <= len) && (len <= 8) )
			m_iLen = len;
	}

	/**
	 * Get the message type. By default, the type is 0x00.
	 * @return The message type.
	 */
	int getType()
	{
		return m_iType;
	}

	/**
	 * Set the message type. By default, the type is 0x00.
	 * @param type The message type.
	 */
	void setType(int type)
	{
		m_iType = type;
	}


};
//-----------------------------------------------
#endif
