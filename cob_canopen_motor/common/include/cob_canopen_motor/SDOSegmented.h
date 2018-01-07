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


#ifndef _SDOSegmented_H
#define _SDOSegmented_H

#include <vector>

/**
* This class is used to collect data that is uploaded to the master in an segmented SDO transfer. Additionally, it includes some administrative functions for this proccess.
* It can be seen as a SDO segmented collector.
*/
class segData {

	public:

		/**
		* States, that are used to describe the current state of the transmission process of the collected segmented SDO transfer.
		*/
		enum SDOStatusFlag {
			SDO_SEG_FREE = 0, /**< SDO collector is ready for a new transmission */
			SDO_SEG_WAITING = 3, /**< SDO collector is waiting for the first bytes to receive */
			SDO_SEG_COLLECTING = 2, /**< SDO collector is currently collecting data in a segmented SDO transfer */
			SDO_SEG_PROCESSING = 1, /**< collection of data is finished but still has to be processed */
		};

		segData() {
			objectID = 0x0000;
			objectSubID = 0x00;
			toggleBit = false;
			statusFlag = SDO_SEG_FREE;
		}

		~segData() {}

		/**
		* Clear the SDO segmented collector
		*/
		void resetTransferData() {
			data.clear();
			objectID = 0x0000;
			objectSubID = 0x00;
			toggleBit = false;
			statusFlag = SDO_SEG_FREE;
		}

		//public attributes
		//all attributes are public, as this class is used only as ~data array

		/**
		* combines different status flags and represents the workflow from 3 to 0:
		*	3: SDORequest sent, waiting for transmission !If you are expecting a Segmented answer, this must be set during the request!
		*	2: SDO process initiated, collecting data
		*	1: finished transmission, waiting for data processing
		*	0: SDO workflow finished, free for new transmission
		*/
		int statusFlag;

		/**
		* Holds the ID of the currently uploading object
		*/
		int objectID;

		/**
		* Holds the Sub-ID of the currently uploading object
		*/
		int objectSubID;

		/**
		* The toggle bit, that has to be alternated in each confirmation response to a received segment.
		*/
		bool toggleBit;

		/**
		* Contains the total number of bytes to be uploaded (if specified by SDO sehmented header)
		*/
		unsigned int numTotalBytes;

		/**
		* This vector holds the received data byte-wise
		*/
		std::vector<unsigned char> data;
};

#endif
