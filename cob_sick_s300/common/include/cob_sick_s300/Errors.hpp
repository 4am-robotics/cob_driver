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
 

#ifndef BRICS_OODL_ERRORS_H
#define BRICS_OODL_ERRORS_H


#include <map>
using namespace std;

#include <string>
#include <iostream>
#include "Logger.hpp"
namespace brics_oodl {

/**
 * \brief
 *
 */
class Errors {
  public:
    Errors();

    virtual ~Errors();

    void getNextError(std::string& name, std::string& description);

    void getAllErrors(map<std::string, std::string>& allErrors);

    unsigned int getAmountOfErrors();

    void addError(std::string name, std::string description);

    void deleteAllErrors();

    void printErrorsToConsole();


  private:
    //map of error name and error description
    map<std::string, std::string> occurredErrors;

    map<std::string,std::string>::iterator iter;

};

} // namespace brics_oodl
#endif
