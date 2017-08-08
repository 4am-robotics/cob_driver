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
 

#ifndef _SerialIO_H
#define _SerialIO_H

#include <termios.h>
#include <sys/select.h>

#include <string>
#include <string.h>

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
	int openIO();

	/**
	 * Closes the serial port.
	 */
	void closeIO();

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
	int writeIO(const char *Buffer, int Length);

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

