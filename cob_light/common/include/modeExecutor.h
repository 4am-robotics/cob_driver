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

#ifndef MODEEXECUTOR_H
#define MODEEXECUTOR_H

#include <mode.h>
#include <iColorO.h>
#include <modeFactory.h>
#include <boost/thread.hpp>
#include <boost/lambda/bind.hpp>
#include <map>

class ModeExecutor
{
public:
	ModeExecutor(IColorO* colorO);
	~ModeExecutor();

	uint64_t execute(boost::shared_ptr<Mode> mode);
	uint64_t execute(cob_light::LightMode requestMode);

	int getExecutingPriority();
	int getExecutingMode();
	uint64_t getExecutingUId();

	void pause();
	void resume();
	void stop();
	bool stop(uint64_t uId);

	void setDefaultPriority(int priority);

private:
	IColorO* _colorO;

	boost::shared_ptr<Mode> _activeMode;
	std::map<int, boost::shared_ptr<Mode>, std::greater<int> > _mapActiveModes;
	color::rgba _activeColor;

	bool _stopRequested;
	int default_priority;

	void onModeFinishedReceived(int prio);
	void onColorSetReceived(color::rgba color);
};

#endif
