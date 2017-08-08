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
 

#ifndef BRICS_OODL_LOGGER_HPP
#define	BRICS_OODL_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <ros/ros.h>

#include "boost/date_time/posix_time/posix_time.hpp"


namespace brics_oodl {

    enum severity_level {
        trace,
        debug,
        info,
        warning,
        error,
        fatal
    };

    ///////////////////////////////////////////////////////////////////////////////
    /// Implementation logging to console and to a file
    ///////////////////////////////////////////////////////////////////////////////
    class Logger {
    private:
        std::stringstream out;
        bool print;
        severity_level level;
    public:

        Logger(const std::string &funcName, const int &lineNo, const std::string &fileName, severity_level level);
        ~Logger();

        static bool toConsole;
        static bool toFile;
        static bool toROS;
        static severity_level logginLevel;

        template <class T>
        Logger & operator<<(const T &v) {
            out << v;
            return *this;
        }
    };


#define LOG(level) Logger(__PRETTY_FUNCTION__, __LINE__ , __FILE__, level)


} // namespace youbot

#endif	/* YOUBOT_LOGGER_HPP */

