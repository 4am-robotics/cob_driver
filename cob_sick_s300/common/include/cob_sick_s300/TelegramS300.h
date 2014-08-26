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
 * ROS package name: cob_sick_s300
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Christian Connette, email:christian.connette@ipa.fhg.de
 * Supervised by: Christian Connette, email:christian.connette@ipa.fhg.de
 *
 * Date of creation: Jan 2009
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

#pragma once

#include <arpa/inet.h>

class TelegramParser {

	#pragma pack(push,1)
	union TELEGRAM_COMMON1 {
		struct {
			uint32_t reply_telegram;
			uint16_t trigger_result;
			uint16_t size;
			uint8_t  coordination_flag;
			uint8_t  device_addresss;
		};
		uint8_t bytes[10];
	};
	union TELEGRAM_COMMON2 {
		struct {
			uint16_t protocol_version;
			uint16_t status;
			uint32_t scan_number;
			uint16_t telegram_number;
		};
		uint8_t bytes[10];
	};
	union TELEGRAM_COMMON3 {
		struct {
			uint16_t type;
		};
		uint8_t bytes[2];
	};

	union TELEGRAM_DISTANCE {
		struct {
			uint16_t type;
		};
		uint8_t bytes[2];
	};

	union TELEGRAM_TAIL {
		struct {
			uint16_t crc;
		};
		uint8_t bytes[2];
	};

	union TELEGRAM_S300_DIST_2B {
		struct {
			unsigned distance : 13; //cm
			unsigned bit13 : 1;  //reflector or scanner distorted
			unsigned protective : 1;
			unsigned warn_field : 1;
		};
		uint16_t val16;
		uint8_t bytes[2];
	};

	#pragma pack(pop)

	enum TELEGRAM_COMMON_HS {JUNK_SIZE=4};
	enum TELEGRAM_COMMON_TYPES {IO=0xAAAA, DISTANCE=0xBBBB, REFLEXION=0xCCCC};
	enum TELEGRAM_DIST_SECTOR {_1=0x1111, _2=0x2222, _3=0x3333, _4=0x4444, _5=0x5555};


	static void ntoh(TELEGRAM_COMMON1 &tc) {
		tc.reply_telegram = ntohl(tc.reply_telegram);
		tc.trigger_result = ntohs(tc.trigger_result);
		tc.size = ntohs(tc.size);
	}

	static void ntoh(TELEGRAM_COMMON2 &tc) {
		tc.protocol_version = ntohs(tc.protocol_version);
		tc.status = ntohs(tc.status);
		tc.scan_number = ntohl(tc.scan_number);
		tc.telegram_number = ntohs(tc.telegram_number);
	}

	static void ntoh(TELEGRAM_COMMON3 &tc) {
		tc.type = ntohs(tc.type);
	}

	static void ntoh(TELEGRAM_DISTANCE &tc) {
		tc.type = ntohs(tc.type);
	}

	static void ntoh(TELEGRAM_TAIL &tc) {
	    //crc calc. is also in network order
		//tc.crc = ntohs(tc.crc);
	}

	static void print(const TELEGRAM_COMMON1 &tc) {
		std::cout<<"HEADER"<<std::dec<<std::endl;
		std::cout<<"reply_telegram"<<":"<<tc.reply_telegram<<std::endl;
		std::cout<<"trigger_result"<<":"<<tc.trigger_result<<std::endl;
		std::cout<<"size"<<":"<<2*tc.size<<std::endl;
		std::cout<<"coordination_flag"<<":"<< std::hex<<tc.coordination_flag<<std::endl;
		std::cout<<"device_addresss"<<":"<< std::hex<<(int)tc.device_addresss<<std::endl;
	}

	static void print(const TELEGRAM_COMMON2 &tc) {
		std::cout<<"protocol_version"<<":"<< std::hex<<tc.protocol_version<<std::endl;
		std::cout<<"status"<<":"<<tc.status<<std::endl;
		std::cout<<"scan_number"<<":"<< std::hex<<tc.scan_number<<std::endl;
		std::cout<<"telegram_number"<<":"<< std::hex<<tc.telegram_number<<std::endl;
	}

	static void print(const TELEGRAM_COMMON3 &tc) {
		std::cout<<"type"<<":"<< std::hex<<tc.type<<std::endl;
		switch(tc.type) {
			case IO: std::cout<<"type"<<": "<<"IO"<<std::endl; break;
			case DISTANCE: std::cout<<"type"<<": "<<"DISTANCE"<<std::endl; break;
			case REFLEXION: std::cout<<"type"<<": "<<"REFLEXION"<<std::endl; break;
			default: std::cout<<"type"<<": "<<"unknown "<<tc.type<<std::endl; break;
		}
		std::cout<<std::dec<<std::endl;
	}

	static void print(const TELEGRAM_DISTANCE &tc) {
		std::cout<<"DISTANCE"<<std::endl;
		std::cout<<"type"<<":"<< std::hex<<tc.type<<std::endl;
		switch(tc.type) {
			case _1: std::cout<<"field 1"<<std::endl; break;
			case _2: std::cout<<"field 2"<<std::endl; break;
			case _3: std::cout<<"field 3"<<std::endl; break;
			case _4: std::cout<<"field 4"<<std::endl; break;
			case _5: std::cout<<"field 5"<<std::endl; break;
			default: std::cout<<"unknown "<<tc.type<<std::endl; break;
		}
		std::cout<<std::dec<<std::endl;
	}

	static void print(const TELEGRAM_TAIL &tc) {
		std::cout<<"TAIL"<<std::endl;
		std::cout<<"crc"<<":"<< std::hex<<tc.crc<<std::endl;
		std::cout<<std::dec<<std::endl;
	}

	//-------------------------------------------
	static unsigned int createCRC(uint8_t *ptrData, int Size);

	//supports versions: 0301, 0201
	static bool check(const TELEGRAM_COMMON1 &tc, const uint8_t DEVICE_ADDR) {
		uint8_t TELEGRAM_COMMON_PATTERN_EQ[] = {0,0,0,0, 0,0, 0,0, 0xFF, 0&DEVICE_ADDR/*version, 2, 1*/};
		uint8_t TELEGRAM_COMMON_PATTERN_OR[] = {0,0,0,0, 0,0, 0xff,0xff, 0,0xff/*version, 1, 0*/};

		for(size_t i=0; i<sizeof(TELEGRAM_COMMON_PATTERN_EQ); i++) {
			if(TELEGRAM_COMMON_PATTERN_EQ[i] != (tc.bytes[i]&(~TELEGRAM_COMMON_PATTERN_OR[i])) ) {
				//std::cout<<"invalid at byte "<<i<<std::endl;
				return false;
			}
		}

		return true;
	}

	TELEGRAM_COMMON1 tc1_;
	TELEGRAM_COMMON2 tc2_;
	TELEGRAM_COMMON3 tc3_;
	TELEGRAM_DISTANCE td_;
	int last_offset_;
public:

	TelegramParser() : last_offset_(0) {}

	bool parseHeader(const unsigned char *buffer, const size_t max_size, const uint8_t DEVICE_ADDR, const bool debug)
	{
		if(sizeof(tc1_)>max_size) return false;
		tc1_ = *((TELEGRAM_COMMON1*)buffer);

		if(!check(tc1_, DEVICE_ADDR)) {
			//if(debug) std::cout<<"basic check failed"<<std::endl;
			return false;
		}

		ntoh(tc1_);
		if(debug) print(tc1_);

		if(tc1_.size*2+JUNK_SIZE>(int)max_size) {
			if(debug) std::cout<<"invalid header size"<<std::endl;
			return false;
		}

		tc2_ = *((TELEGRAM_COMMON2*)(buffer+sizeof(TELEGRAM_COMMON1)));
		tc3_ = *((TELEGRAM_COMMON3*)(buffer+(sizeof(TELEGRAM_COMMON1)+sizeof(TELEGRAM_COMMON2))));

		TELEGRAM_TAIL tt;
		uint16_t crc;

		if(tc2_.protocol_version==0x102)
			last_offset_ = -10;
		tt = *((TELEGRAM_TAIL*) (buffer+(2*tc1_.size+JUNK_SIZE-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_)) );
		ntoh(tt);
		crc = createCRC((uint8_t*)buffer+JUNK_SIZE, 2*tc1_.size-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_);
		
		if(tc2_.protocol_version!=0x102) {
			if(tt.crc!=crc) {
				last_offset_ = 0;
				if(debug) std::cout<<"CRC failed!\ntrying offset of "<<std::dec<<last_offset_<<std::endl;
				tt = *((TELEGRAM_TAIL*) (buffer+(2*tc1_.size+JUNK_SIZE-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_)) );
				ntoh(tt);
				crc = createCRC((uint8_t*)buffer+JUNK_SIZE, 2*tc1_.size-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_);
			}

			if(tt.crc!=crc) {
				last_offset_ = -6;
				if(debug) std::cout<<"CRC failed!\ntrying offset of "<<std::dec<<last_offset_<<std::endl;
				tt = *((TELEGRAM_TAIL*) (buffer+(2*tc1_.size+JUNK_SIZE-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_)) );
				ntoh(tt);
				crc = createCRC((uint8_t*)buffer+JUNK_SIZE, 2*tc1_.size-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_);
			}
		}

		if(tt.crc!=crc) {
			if(debug) {
				print(tc2_);
				print(tc3_);
				print(tt);
				std::cout<<"at "<<std::hex<<(2*tc1_.size+JUNK_SIZE-sizeof(TELEGRAM_TAIL)+sizeof(TELEGRAM_COMMON2)+last_offset_)<<std::endl;
				std::cout<<"invalid CRC: "<<crc<<" ("<<tt.crc<<")"<<std::endl;
			}
			return false;
		}

		memset(&td_, 0, sizeof(td_));
		switch(tc3_.type) {
			case IO: break;

			case DISTANCE:
				if(debug) std::cout<<"got distance"<<std::endl;

				td_ = *((TELEGRAM_DISTANCE*)(buffer+sizeof(tc1_)+sizeof(tc2_)+sizeof(tc3_)));
				ntoh(td_);
				//print(td_);
				break;

			case REFLEXION: break;
			default: return false;
		}

		return true;
	}

	bool isDist() const {return tc3_.type==DISTANCE;}
	int getField() const {
		switch(td_.type) {
			case _1: return 1;
			case _2: return 2;
			case _3: return 3;
			case _4: return 4;
			case _5: return 5;
			default: return -1;
		}
	}

	int getCompletePacketSize() const {
		return 2*tc1_.size + sizeof(tc1_) + JUNK_SIZE + last_offset_;
	}

	void readDistRaw(const unsigned char *buffer, std::vector<int> &res) const
	{
		res.clear();
		if(!isDist()) return;

		size_t num_points = (2*tc1_.size - (sizeof(tc1_)+sizeof(tc2_)+sizeof(tc3_)+sizeof(td_)+sizeof(TELEGRAM_TAIL)-JUNK_SIZE-last_offset_));
		//std::cout<<"num_points: "<<std::dec<<num_points/sizeof(TELEGRAM_S300_DIST_2B)<<"  "<<num_points<<" "<<tc1_.size<<std::endl;
		size_t i=0;
		for(; i<num_points; ) {
			TELEGRAM_S300_DIST_2B dist = *((TELEGRAM_S300_DIST_2B*) (buffer+(sizeof(tc1_)+sizeof(tc2_)+sizeof(tc3_)+sizeof(td_)+i)) );
			//for distance only: res.push_back((int)dist.distance);
			res.push_back((int)dist.val16);
			i += sizeof(TELEGRAM_S300_DIST_2B);
		}
	}

};
