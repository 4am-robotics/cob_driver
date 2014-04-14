/****************************************************************
 *
 * Copyright (c) 2011
 * All rights reserved.
 *
 * Hochschule Bonn-Rhein-Sieg
 * University of Applied Sciences
 * Computer Science Department
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author:
 * Jan Paulus, Nico Hochgeschwender, Michael Reckhaus, Azamat Shakhimardanov
 * Supervised by:
 * Gerhard K. Kraetzschmar
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * This sofware is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
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
 *     * Neither the name of the Hochschule Bonn-Rhein-Sieg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ****************************************************************/

#include "cob_sick_s300/Logger.hpp"

namespace brics_oodl {

  bool Logger::toConsole = false;
  bool Logger::toFile = false;
  bool Logger::toROS = true;
  severity_level Logger::logginLevel = trace;

  Logger::Logger(const std::string &funcName, const int &lineNo, const std::string &fileName, severity_level level) {

    this->level = level;
    if (toConsole || toFile) {
      if (level >= logginLevel) {
        print = true;

        switch (level) {
          case trace:
            out << "Trace" << ": ";
            break;
          case debug:
            out << "Debug" << ": ";
            break;
          case info:
            out << "Info" << ": ";
            break;
          case warning:
            out << "Warning" << ": ";
            break;
          case error:
            out << "Error" << ": ";
            break;
          case fatal:
            out << "Fatal" << ": ";
            break;
          default:
            break;
        }
        //  out << "function " << funcName << ": ";
        //  out << "line " << lineNo << ": ";
        //  out << "fileName " << fileName << ": ";
        //  out << "time " << boost::posix_time::microsec_clock::local_time() << ": ";
      } else {
        print = false;
      }
    } else {
      print = false;
    }

  }

  Logger::~Logger() {
    //end of message
    if (toConsole && print) {
      std::cout << out.str() << std::endl;
    }

    if (toFile && print) {
      std::fstream filestr;
      filestr.open("log.txt", std::fstream::out | std::fstream::app);
      filestr << out.str() << std::endl;
      filestr.close();
    }

    if (toROS) {
      switch (level) {
        case trace:
          ROS_DEBUG(out.str().c_str());
          break;
        case debug:
          ROS_DEBUG(out.str().c_str());
          break;
        case info:
          ROS_INFO(out.str().c_str());
          break;
        case warning:
          ROS_WARN(out.str().c_str());
          break;
        case error:
          ROS_ERROR(out.str().c_str());
          break;
        case fatal:
          ROS_FATAL(out.str().c_str());
          break;
        default:
          break;
      }
    }
  }

} // namespace youbot
