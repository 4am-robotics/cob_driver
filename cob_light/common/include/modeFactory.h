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


#ifndef MODEFACTORY_H
#define MODEFACTORY_H

#include <mode.h>
#include <iColorO.h>
#include <cob_light/SetLightMode.h>
#include <cob_light/LightModes.h>
#include <boost/shared_ptr.hpp>

class ModeFactory
{
public:
	ModeFactory();
	~ModeFactory();

	static boost::shared_ptr<Mode> create(cob_light::LightMode requestMode, IColorO* colorO);
	static boost::shared_ptr<Mode> create(std::string mode, color::rgba color);

	static int type(Mode *mode);

private:
	IColorO* _colorO;
};

#endif
