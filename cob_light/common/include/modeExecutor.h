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
