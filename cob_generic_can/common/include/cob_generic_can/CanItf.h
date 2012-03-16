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
#ifndef CANITF_INCLUDEDEF_H
#define CANITF_INCLUDEDEF_H

//-----------------------------------------------

#include <cob_generic_can/CanMsg.h>
#include <cob_generic_can/stdDef.h>

//-----------------------------------------------

/**
 * General interface of the CAN bus.
 * \ingroup DriversCanModul	
 */
class CanItf
{
public:

	enum BaudRate
	{
		CAN_BAUDRATE_1M = 0,
		CAN_BAUDRATE_500K = 1,
		CAN_BAUDRATE_250K = 2,
		CAN_BAUDRATE_125K = 3,
		CAN_BAUDRATE_100K = 4,
		CAN_BAUDRATE_50K = 5,
		CAN_BAUDRATE_20K = 6,
		CAN_BAUDRATED_10K = 7,
		CAN_BAUDRATE_5K = 8
	};

	/**
	 *
	 */
	virtual ~CanItf() { }

	/**
	 * Initializes the CAN bus.
	 */
	virtual void init(int iBaudRate) = 0;

	/**
	 * Sends a CAN message.
	 */
	virtual bool transmitMsg(CanMsg& CMsg) = 0;

	/**
	 * Reads a CAN message.
	 * @return true if a message is available 
	 */
	virtual bool receiveMsg(CanMsg* pCMsg) = 0;

	/**
	 * Reads a CAN message.
	 * The function blocks between the attempts.
	 * @param pCMsg CAN message
	 * @param iNrOfRetry number of retries
	 * @return true if a message is available
	 */
	virtual bool receiveMsgRetry(CanMsg* pCMsg, int iNrOfRetry) = 0;

	/**
	 * Set Em-stop in StateArm Active.
	 * Only necessary for arm on mx500 because there is 
	 * no possibility to recognize EM-Stop except on Can-Errors
	 */
	virtual bool emMessageError() = 0;
};
//-----------------------------------------------
#endif
