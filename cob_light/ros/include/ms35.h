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


#ifndef ES35_H
#define ES35_H

#include <iColorO.h>

#include <serialIO.h>
#include <colorUtils.h>
#include <sstream>

class MS35 : public IColorO
{
public:
  MS35(SerialIO* serialIO);
  virtual ~MS35();

  bool init();
  void setColor(color::rgba color);
  void setColorMulti(std::vector<color::rgba> &colors);

private:
  SerialIO* _serialIO;
  std::stringstream _ssOut;
  static const int PACKAGE_SIZE = 9;
  char buffer[PACKAGE_SIZE];

  int sendData(const char* data, size_t len);
  unsigned short int getChecksum(const char* data, size_t len);
};

#endif
