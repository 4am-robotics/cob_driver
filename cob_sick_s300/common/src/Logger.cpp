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
          ROS_DEBUG("%s",out.str().c_str());
          break;
        case debug:
          ROS_DEBUG("%s",out.str().c_str());
          break;
        case info:
          ROS_INFO("%s",out.str().c_str());
          break;
        case warning:
          ROS_WARN("%s",out.str().c_str());
          break;
        case error:
          ROS_ERROR("%s",out.str().c_str());
          break;
        case fatal:
          ROS_FATAL("%s",out.str().c_str());
          break;
        default:
          break;
      }
    }
  }

} // namespace youbot
