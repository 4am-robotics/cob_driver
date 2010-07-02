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
 * ROS package name: cob_canopen_motor
 * Description: Holds data, that is collected during a SDO Segmented Upload process
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Philipp Köhler
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Mar 2010
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

#ifndef _RecorderData_H
#define _RecorderData_H

#include <vector>

/** Measure system time.
 * Use this class for measure system time accurately. Under Windows, it uses
 * QueryPerformanceCounter(), which has a resolution of approx. one micro-second.
 * The difference between two time stamps can be calculated.
 */

class recData {
    public:
        
        recData() {
            bytesReceived = 0;
            finishedTransmission = false;
            locked = false;
            objectID = 0x00;
            objectSubID = 0x00;
            }

        ~recData() {}

        void resetTransferData() {
            if (locked == false) {
                bytesReceived = 0;
                data.clear();
                finishedTransmission = false;
                objectID = 0x00;
                objectSubID = 0x00;
            }
        }
            
        
        unsigned int numTotalBytes; //contains the number of bytes to be uploaded (if specified)

        int bytesReceived; //number of data bytes already received in current SDO Upload process      

        bool finishedTransmission; //no more segments to receive

        bool locked; //prevent Data from beeing resetted before read out has been proceeded

        int objectID;
        int objectSubID;

        std::vector<unsigned char> data; //this vector holds received bytes as a stream. Little endian conversion is already done during receive. 

};

#endif
