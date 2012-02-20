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

