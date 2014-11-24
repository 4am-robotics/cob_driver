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
 * ROS package name: cob_light
 * Description: Switch robots led color by sending data to
 * the led-µC over serial connection.
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 * Supervised by: Benjamin Maidel, email:benjamin.maidel@ipa.fraunhofer.de
 *
 * Date of creation: August 2012
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

#include <modeFactory.h>
#include <staticMode.h>
#include <flashMode.h>
#include <breathMode.h>
#include <breathColorMode.h>
#include <fadeColorMode.h>


ModeFactory::ModeFactory()
{
}
ModeFactory::~ModeFactory()
{
}

Mode* ModeFactory::create(cob_light::LightMode requestMode)
{
	Mode* mode = NULL;
	color::rgba color;
	color.r = requestMode.color.r;
	color.g = requestMode.color.g;
	color.b = requestMode.color.b;
	color.a = requestMode.color.a;

	switch(requestMode.mode)
	{
		case cob_light::LightMode::STATIC:
			mode = new StaticMode(color, requestMode.priority, requestMode.frequency,\
				requestMode.pulses, requestMode.timeout);
		break;

		case cob_light::LightMode::FLASH:
			mode = new FlashMode(color, requestMode.priority, requestMode.frequency,\
				requestMode.pulses, requestMode.timeout);
		break;

		case cob_light::LightMode::BREATH:
			mode = new BreathMode(color, requestMode.priority, requestMode.frequency,\
				requestMode.pulses, requestMode.timeout);
		break;

		case cob_light::LightMode::BREATH_COLOR:
			mode = new BreathColorMode(color, requestMode.priority, requestMode.frequency,\
				requestMode.pulses, requestMode.timeout);
		break;

		case cob_light::LightMode::FADE_COLOR:
			mode = new FadeColorMode(color, requestMode.priority, requestMode.frequency,\
				requestMode.pulses, requestMode.timeout);
		break;

		default:
			mode = NULL;
	}

	return mode;
}

Mode* ModeFactory::create(std::string requestMode, color::rgba color)
{
	Mode* mode = NULL;

	if(requestMode == "Static" || requestMode == "static" || requestMode == "STATIC")
	{
			mode = new StaticMode(color);
	}
	else if(requestMode == "Flash" || requestMode == "flash" || requestMode == "FLASH")
	{
			mode = new FlashMode(color);
	}
	else if(requestMode == "Breath" || requestMode == "breath" || requestMode == "BREATH")
	{
			mode = new BreathMode(color);
	}
	else if(requestMode == "BreathColor" || requestMode == "BreathColor" || requestMode == "BreathColor" ||
		requestMode == "Breath_Color" || requestMode == "breath_color" || requestMode == "BREATH_COLOR")
	{
			mode = new BreathColorMode(color);
	}
	else if(requestMode == "FadeColor" || requestMode == "fadecolor" || requestMode == "FADECOLOR" ||
		requestMode == "Fade_Color" || requestMode == "fade_color" || requestMode == "FADE_COLOR")
	{
			mode = new FadeColorMode(color);
	}
	else
	{
		mode = NULL;
	}

	return mode;
}

int ModeFactory::type(Mode *mode)
{
	int ret;
	if (mode == NULL)
		ret = cob_light::LightMode::NONE;
	else if(dynamic_cast<StaticMode*>(mode) != NULL)
		ret = cob_light::LightMode::STATIC;
	else if(dynamic_cast<FlashMode*>(mode) != NULL)
		ret = cob_light::LightMode::FLASH;
	else if(dynamic_cast<BreathMode*>(mode) != NULL)
		ret = cob_light::LightMode::BREATH;
	else if(dynamic_cast<BreathColorMode*>(mode) != NULL)
		ret = cob_light::LightMode::BREATH_COLOR;
	else if(dynamic_cast<FadeColorMode*>(mode) != NULL)
		ret = cob_light::LightMode::FADE_COLOR;
	else
		ret = cob_light::LightMode::NONE;

	return ret;
}