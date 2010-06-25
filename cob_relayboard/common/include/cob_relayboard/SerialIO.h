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
 * ROS stack name: cob_driver
 * ROS package name: cob_relayboard
 * Description: Class for communication with relayboard. The relayboard is mainly used for reading the Emergencystop and Laserscannerstop states.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Koehler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: March 2010
 * ToDo:
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

#ifndef _SerialIO_H
#define _SerialIO_H

#include <termios.h>
#include <sys/select.h>

#include <string>

/**
 * Wrapper class for serial communication.
 */
class SerialIO  
{
public:
	// ---------------------- Constants
	/// Constants for defining the handshake.
	enum HandshakeFlags
	{
		HS_NONE,
		HS_HARDWARE,
		HS_XONXOFF
	};

	/// Constants for defining the parity bits.
	enum ParityFlags 
	{ 
		PA_NONE, 
		PA_EVEN,
		PA_ODD,
// UNIX serial drivers only support even, odd, and no parity bit generation.
 		PA_MARK,
		PA_SPACE
	};

	/// Constants for defining the stop bits.
	enum StopBits
	{ 
		SB_ONE, 
		SB_ONE_5, // ????? returns an error ?????
		SB_TWO 
	};

	/// Default constructor
	SerialIO();

	/// Destructor
	virtual ~SerialIO();

	/**
	 * Sets the device name
	 * @param Name 'COM1', 'COM2', ...
	 */
	void setDeviceName(const char *Name) { m_DeviceName = Name; }

	/**
	 * Sets the baudrate.
	 * @param BaudRate baudrate.
	 */
	void setBaudRate(int BaudRate) { m_BaudRate = BaudRate; }

	/**
	 * Changes the baudrate.
	 * The serial port is allready open.
	 * @param BaudRate new baudrate.
	 */
	void changeBaudRate(int BaudRate);

	/**
	 * Sets a multiplier for the baudrate.
	 * Some serial cards need a specific multiplier for the baudrate.
	 * @param Multiplier default is one.
	 */
	void setMultiplier(double Multiplier = 1) { m_Multiplier = Multiplier; };

	/**
	 * Sets the message format.
	 */
	void SetFormat(int ByteSize, ParityFlags Parity, int StopBits)
		{ m_ByteSize = ByteSize; m_Parity = Parity; m_StopBits = StopBits; }

	/**
	 * Defines the handshake type.
	 */
	void setHandshake(HandshakeFlags Handshake) { m_Handshake = Handshake; }

	/**
	 * Sets the buffer sizes.
	 * @param ReadBufSize number of bytes of the read buffer.
	 * @param WriteBufSize number of bytes of the write buffer.
	 */
	void setBufferSize(int ReadBufSize, int WriteBufSize)
		{ m_ReadBufSize = ReadBufSize; m_WriteBufSize = WriteBufSize; }

	/**
	 * Sets the timeout.
	 * @param Timeout in seconds
	 */
	void setTimeout(double Timeout);

	/**
	 * Sets the byte period for transmitting bytes.
	 * If the period is not equal to 0, the transmit will be repeated with the given
	 * period until all bytes are transmitted.
	 * @param default is 0.
	 */
	void setBytePeriod(double Period);

	/**
	 * Opens serial port.
	 * The port has to be configured before.
	 */
	int open();

	/**
	 * Closes the serial port.
	 */
	void close();

	/**
	 * Reads the serial port blocking.
	 * The function blocks until the requested number of bytes have been
	 * read or the timeout occurs.
	 * @param Buffer pointer to the buffer.
	 * @param Length number of bytes to read
	 */
	int readBlocking(char *Buffer, int Length);


	/**
	 * Reads the serial port non blocking.
	 * The function returns all avaiable bytes but not more than requested.
	 * @param Buffer pointer to the buffer.
	 * @param Length number of bytes to read
	 */
	int readNonBlocking(char *Buffer, int Length);

	/**
	 * Writes bytes to the serial port.
	 * @param Buffer buffer of the message
	 * @param Length number of bytes to send
	 */
	int write(const char *Buffer, int Length);

	/**
	 * Returns the number of bytes available in the read buffer.
	 */
	int getSizeRXQueue();


	/** Clears the read and transmit buffer.
	 */
	void purge()
	{
		::tcflush(m_Device, TCIOFLUSH);
	}

	/** Clears the read buffer.
	 */
	void purgeRx() {
		tcflush(m_Device, TCIFLUSH);
}

	/**
	 * Clears the transmit buffer.
	 * The content of the buffer will not be transmitted.
	 */ 
	void purgeTx() {
		tcflush(m_Device, TCOFLUSH);
	}

	/** 
	 * Sends the transmit buffer.
	 * All bytes of the transmit buffer will be sent.
	 */
	void flushTx() {
		tcdrain(m_Device);
	}

protected:
	::termios m_tio;
	std::string m_DeviceName;
	int m_Device;
	int m_BaudRate;
	double m_Multiplier;
	int m_ByteSize, m_StopBits;
	ParityFlags m_Parity;
	HandshakeFlags m_Handshake;
	int m_ReadBufSize, m_WriteBufSize;
	double m_Timeout;
	::timeval m_BytePeriod;
	bool m_ShortBytePeriod;
};


#endif //

